#include <wx/wx.h>
#include <wx/dirdlg.h>
#include <wx/filedlg.h>
#include <wx/listbox.h>
#include <wx/sizer.h>
#include <wx/textctrl.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <pugixml.hpp>

#include "utils.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

constexpr double kCorrectionThresholdRadians = 1.3;
constexpr float kCorrectionOffset = 0.60230f;

struct Config {
    fs::path personalRoot;
    fs::path officialRoot;
    fs::path outputRoot = fs::current_path();
};

class Processor {
public:
    using Logger = std::function<void(const std::string&)>;

    Processor(Config config, Logger logger)
        : config_(std::move(config)), logger_(std::move(logger)) {}

    void processInput(const fs::path& input) {
        const fs::path absolute = fs::absolute(input).lexically_normal();
        fs::path relative;

        if (!config_.personalRoot.empty() &&
            isWithin(absolute, fs::absolute(config_.personalRoot).lexically_normal())) {
            relative = absolute.lexically_relative(
                fs::absolute(config_.personalRoot).lexically_normal());
        } else if (!config_.officialRoot.empty() &&
                   isWithin(absolute, fs::absolute(config_.officialRoot).lexically_normal())) {
            relative = absolute.lexically_relative(
                fs::absolute(config_.officialRoot).lexically_normal());
        } else {
            throw std::runtime_error(
                "Eingabedatei ist nicht im eigenen oder offiziellen Datenverzeichnis: " +
                absolute.string());
        }

        processFile(relative, glm::mat4(1.0f));
    }

private:
    Config config_;
    Logger logger_;
    std::set<std::string> processed_;

    static bool isWithin(const fs::path& path, const fs::path& root) {
        const auto rel = path.lexically_relative(root);
        if (rel.empty()) return path == root;
        return *rel.begin() != "..";
    }

    static std::string keyFor(const fs::path& path) {
        std::string key = path.lexically_normal().generic_string();
#ifdef _WIN32
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
#endif
        return key;
    }

    static glm::vec3 readVec3(const pugi::xml_node& node) {
        if (!node) return glm::vec3(0.0f);
        return {
            node.attribute("X").as_float(0.0f),
            node.attribute("Y").as_float(0.0f),
            node.attribute("Z").as_float(0.0f)
        };
    }

    static glm::mat4 transformFromPositionEuler(
        const glm::vec3& position, const glm::vec3& euler) {
        // The original Python used intrinsic ZYX rotations: Rz * Ry * Rx.
        const glm::quat qz = glm::angleAxis(euler.z, glm::vec3(0, 0, 1));
        const glm::quat qy = glm::angleAxis(euler.y, glm::vec3(0, 1, 0));
        const glm::quat qx = glm::angleAxis(euler.x, glm::vec3(1, 0, 0));

        glm::mat4 result = glm::mat4_cast(qz * qy * qx);
        result[3] = glm::vec4(position, 1.0f);
        return result;
    }

    static void positionEulerFromTransform(
        const glm::mat4& transform, glm::vec3& position, glm::vec3& euler) {
        position = glm::vec3(transform[3]);
        euler = glm::eulerAngles(glm::quat_cast(glm::mat3(transform)));
    }

    static double angleBetween(const glm::vec3& a, const glm::vec3& b) {
        const double denom = glm::length(a) * glm::length(b);
        if (denom <= 1e-12) {
            throw std::runtime_error("Kann Winkel für einen Vektor der Länge 0 nicht berechnen.");
        }
        const double cosine = glm::dot(a, b) / denom;
        return std::acos(std::clamp(cosine, -1.0, 1.0));
    }

    static std::string formatFloat(double value, int digits) {
        const double rounded = std::round(value * std::pow(10.0, digits)) /
                               std::pow(10.0, digits);
        if (rounded == 0.0) return {};

        std::ostringstream out;
        out << std::fixed << std::setprecision(digits) << rounded;
        std::string text = out.str();

        while (!text.empty() && text.back() == '0') text.pop_back();
        if (!text.empty() && text.back() == '.') text.pop_back();
        return text;
    }

    static void setFloatAttribute(
        pugi::xml_node node, const char* name, double value, int digits) {
        if (!node) return;

        const std::string text = formatFloat(value, digits);
        if (text.empty()) {
            node.remove_attribute(name);
            return;
        }

        auto attr = node.attribute(name);
        if (!attr) attr = node.append_attribute(name);
        attr.set_value(text.c_str());
    }

    fs::path resolveSource(const fs::path& relative) const {
        if (!config_.personalRoot.empty()) {
            const fs::path candidate = config_.personalRoot / relative;
            if (fs::is_regular_file(candidate)) return candidate;
        }
        if (!config_.officialRoot.empty()) {
            const fs::path candidate = config_.officialRoot / relative;
            if (fs::is_regular_file(candidate)) return candidate;
        }
        throw std::runtime_error("Kann verknüpfte Datei nicht finden: " + relative.string());
    }

    fs::path resolveLink(const fs::path& currentRelative, std::string filename) const {
        std::replace(filename.begin(), filename.end(), '\\', '/');
        fs::path link(filename);

        // A filename with a directory component is rooted at the data directory,
        // matching the source script's intended behavior.
        fs::path relative = link.has_parent_path()
            ? link
            : currentRelative.parent_path() / link;

        relative = relative.lexically_normal();
        if (relative.empty() || *relative.begin() == "..") {
            throw std::runtime_error("Verknüpfte Datei außerhalb Datenverzeichnis: " + filename);
        }
        return relative;
    }

    bool processLinkedNode(
        pugi::xml_node node, const fs::path& currentRelative, const glm::mat4& parentTransform) {
        const glm::vec3 position = readVec3(node.child("p"));
        const glm::vec3 euler = readVec3(node.child("phi"));
        const glm::mat4 localTransform = transformFromPositionEuler(position, euler);
        const glm::mat4 worldTransform = parentTransform * localTransform;

        const pugi::xml_node datei = node.child("Datei");
        if (!datei) return false;

        const std::string filename = datei.attribute("Dateiname").as_string();
        if (filename.empty()) return false;

        bool changed = false;
        const std::string lowerFilename = [&] {
            std::string s = filename;
            std::transform(s.begin(), s.end(), s.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return s;
        }();

        const bool targetIsolator =
            lowerFilename == R"(catenary\deutschland\einzelteile_re75-200\isolator.lod.ls3)" ||
            lowerFilename == R"(catenary\deutschland\einzelteile_re75-200\isolatortragseil.lod.ls3)";

        if (targetIsolator) {
            // The source's fourth vector component was 1, but only the rotated
            // direction is used; using a 3D direction avoids homogeneous-vector ambiguity.
            const glm::vec3 up(0.0f, -1.0f, 0.0f);
            const glm::vec3 transformedUp = glm::mat3(worldTransform) * up;
            const double angle = angleBetween(transformedUp, glm::vec3(0, 0, 1));

            if (angle < kCorrectionThresholdRadians) {
                glm::mat4 rotate(1.0f);
                rotate = glm::rotate(rotate, glm::radians(180.0f), glm::vec3(0, 0, 1));

                glm::mat4 translate(1.0f);
                translate = glm::translate(translate, glm::vec3(0, kCorrectionOffset, 0));

                // Preserve the original order: local * rotation * translation.
                const glm::mat4 corrected = localTransform * rotate * translate;
                const glm::mat4 correctedWorld = parentTransform * corrected;
                const glm::vec3 correctedUp = glm::mat3(correctedWorld) * up;
                const double correctedAngle =
                    angleBetween(correctedUp, glm::vec3(0, 0, 1));

                if (correctedAngle < kCorrectionThresholdRadians) {
                    throw std::runtime_error(
                        "Plausibilitätsprüfung nach Korrektur fehlgeschlagen: " + filename);
                }

                glm::vec3 newPosition;
                glm::vec3 newEuler;
                positionEulerFromTransform(corrected, newPosition, newEuler);

                pugi::xml_node p = node.child("p");
                if (!p) p = node.append_child("p");
                pugi::xml_node phi = node.child("phi");
                if (!phi) phi = node.append_child("phi");

                setFloatAttribute(p, "X", newPosition.x, 4);
                setFloatAttribute(p, "Y", newPosition.y, 4);
                setFloatAttribute(p, "Z", newPosition.z, 4);
                setFloatAttribute(phi, "X", newEuler.x, 6);
                setFloatAttribute(phi, "Y", newEuler.y, 6);
                setFloatAttribute(phi, "Z", newEuler.z, 6);

                logger_("Corrected " + filename);
                changed = true;
            }
        }

        const fs::path childRelative = resolveLink(currentRelative, filename);
        processFile(childRelative, worldTransform);
        return changed;
    }

    void processFile(const fs::path& relative, const glm::mat4& parentTransform) {
        const std::string key = keyFor(relative);
        if (!processed_.insert(key).second) return;

        const fs::path source = resolveSource(relative);
        pugi::xml_document document;
        const pugi::xml_parse_result result =
            document.load_file(source.string().c_str(), (pugi::parse_default | pugi::parse_ws_pcdata) & ~(pugi::parse_eol | pugi::parse_wconv_attribute) );

        if (!result) {
            throw std::runtime_error(
                "XML-Parserfehler in " + source.string() + ": " + result.description());
        }

        bool changed = false;
        const pugi::xpath_node_set linkedNodes =
            document.select_nodes(".//Verknuepfte");

        for (const auto& item : linkedNodes) {
            try {
                if (processLinkedNode(item.node(), relative, parentTransform)) {
                    changed = true;
                }
            } catch (const std::exception& e) {
                logger_(std::string("Fehler in ") + relative.string() + ": " + e.what());
            }
        }

        if (changed) {
            const fs::path destination = config_.outputRoot / relative;
            fs::create_directories(destination.parent_path());

            // https://github.com/zeux/pugixml/discussions/645
            pugi::xml_node decl = document.prepend_child(pugi::node_declaration);
            decl.append_attribute("version").set_value("1.0");
            decl.append_attribute("encoding").set_value("UTF-8");
            document.insert_child_after(pugi::node_pcdata, decl).set_value("\n");
            document.append_child(pugi::node_pcdata).set_value("\n");

            if (!document.save_file(destination.string().c_str(), "", pugi::format_raw | pugi::format_write_bom | pugi::format_save_file_text,
                                    pugi::encoding_utf8)) {
                throw std::runtime_error("Kann Ausgabedatei nicht schreiben: " + destination.string());
            }
            logger_("Geschrieben: " + destination.string());
        }
    }
};

class MainFrame final : public wxFrame {
public:
    MainFrame()
        : wxFrame(nullptr, wxID_ANY, "Isolatorenkorrektur",
                  wxDefaultPosition, wxSize(760, 560)) {
        auto* panel = new wxPanel(this);
        auto* outer = new wxBoxSizer(wxVERTICAL);

        auto* roots = new wxFlexGridSizer(2, 3, 6, 6);
        roots->AddGrowableCol(1, 1);

        personal_ = addDirectoryRow(panel, roots, "Eigenes Datenverzeichnis:");
        personal_->SetValue(zusixml::getZusiDatenpfad());
        official_ = addDirectoryRow(panel, roots, "Offizielles Datenverzeichnis:");
        official_->SetValue(zusixml::getZusiDatenpfadOffiziell());
        outer->Add(roots, 0, wxEXPAND | wxALL, 8);

        files_ = new wxListBox(panel, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                               0, nullptr, wxLB_EXTENDED);
        outer->Add(files_, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);

        auto* buttons = new wxBoxSizer(wxHORIZONTAL);
        auto* addButton = new wxButton(panel, wxID_ANY, "LS3-Dateien hinzufügen...");
        auto* removeButton = new wxButton(panel, wxID_ANY, "Ausgewählte entfernen");
        auto* runButton = new wxButton(panel, wxID_ANY, "Start");
        buttons->Add(addButton, 0, wxRIGHT, 6);
        buttons->Add(removeButton, 0, wxRIGHT, 6);
        buttons->AddStretchSpacer();
        buttons->Add(runButton, 0);
        outer->Add(buttons, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);

        output_ = new wxTextCtrl(panel, wxID_ANY, "", wxDefaultPosition, wxSize(-1, 150),
                                 wxTE_MULTILINE | wxTE_READONLY);
        outer->Add(output_, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);

        panel->SetSizer(outer);

        addButton->Bind(wxEVT_BUTTON, &MainFrame::onAddFiles, this);
        removeButton->Bind(wxEVT_BUTTON, &MainFrame::onRemoveFiles, this);
        runButton->Bind(wxEVT_BUTTON, &MainFrame::onRun, this);
    }

private:
    wxTextCtrl* personal_ = nullptr;
    wxTextCtrl* official_ = nullptr;
    wxListBox* files_ = nullptr;
    wxTextCtrl* output_ = nullptr;

    wxTextCtrl* addDirectoryRow(wxWindow* parent, wxFlexGridSizer* sizer,
                                const wxString& label) {
        auto* text = new wxTextCtrl(parent, wxID_ANY);
        auto* browse = new wxButton(parent, wxID_ANY, "Auswählen...");
        sizer->Add(new wxStaticText(parent, wxID_ANY, label), 0, wxALIGN_CENTER_VERTICAL);
        sizer->Add(text, 1, wxEXPAND);
        sizer->Add(browse, 0);

        browse->Bind(wxEVT_BUTTON, [this, text](wxCommandEvent&) {
            wxDirDialog dialog(this, "Datenverzeichnis auswählen");
            if (dialog.ShowModal() == wxID_OK) text->SetValue(dialog.GetPath());
        });
        return text;
    }

    void onAddFiles(wxCommandEvent&) {
        wxFileDialog dialog(this, "LS3-Dateien auswählen", "", "",
                            "LS3-Dateien (*.ls3)|*.ls3|Alle Dateien (*.*)|*.*",
                            wxFD_OPEN | wxFD_MULTIPLE);
        if (dialog.ShowModal() != wxID_OK) return;

        wxArrayString paths;
        dialog.GetPaths(paths);
        for (const auto& path : paths) {
            if (files_->FindString(path) == wxNOT_FOUND) files_->Append(path);
        }
    }

    void onRemoveFiles(wxCommandEvent&) {
        wxArrayInt selections;
        files_->GetSelections(selections);
        for (int i = static_cast<int>(selections.size()) - 1; i >= 0; --i) {
            files_->Delete(static_cast<unsigned int>(selections[i]));
        }
    }

    void onRun(wxCommandEvent&) {
        if (files_->IsEmpty()) {
            return;
        }

        Config config;
        if (!personal_->GetValue().empty()) config.personalRoot = personal_->GetValue().ToStdWstring();
        if (!official_->GetValue().empty()) config.officialRoot = official_->GetValue().ToStdWstring();

        output_->Clear();
        Processor processor(config, [this](const std::string& line) {
            output_->AppendText(wxString::FromUTF8(line) + "\n");
        });

        for (unsigned int i = 0; i < files_->GetCount(); ++i) {
            try {
                const fs::path input(files_->GetString(i).ToStdWstring());
                output_->AppendText("Verarbeite " + files_->GetString(i) + "\n");
                processor.processInput(input);
            } catch (const std::exception& e) {
                output_->AppendText("FEHLER: " + wxString::FromUTF8(e.what()) + "\n");
            }
        }

        output_->AppendText("Verarbeitung abgeschlossen.\n");
    }
};

class App final : public wxApp {
public:
    bool OnInit() override {
        auto* frame = new MainFrame();
        frame->Show();
        return true;
    }
};

wxIMPLEMENT_APP(App);
