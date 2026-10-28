#ifndef ZUSI_PARSER_UTILS_HPP_
#define ZUSI_PARSER_UTILS_HPP_

#include <algorithm>
#include <cassert>
#include <chrono>
#include <exception>
#include <ios>

#ifdef _WIN32
#  include <boost/nowide/iostream.hpp>
#  include <boost/nowide/fstream.hpp>
#else
#  include <iostream>
#  include <fstream>
#endif

#include <string>
#include <string_view>
#include <vector>

#ifdef _WIN32
#  include <windows.h>
#  include <winerror.h>
#  include <boost/nowide/convert.hpp>
#else
#  include <cerrno>
#  include <fcntl.h>
#  include <sys/mman.h>
#  include <sys/stat.h>
#  include <sys/types.h>
#  include <unistd.h>
#endif

#define MMAP_THRESHOLD_BYTES 0

#ifdef _WIN32
namespace io = boost::nowide;
#else
namespace io = std;
#endif

#ifdef ZUSI_PARSER_USE_BOOST_FILESYSTEM
  #include <boost/filesystem.hpp>
  namespace fs = boost::filesystem;
#else
  #include <filesystem>
  namespace fs = std::filesystem;
#endif

namespace zusixml {

static inline std::string bestimmeZusiDatenpfad() {
  std::string result;
#ifdef _WIN32
  HKEY key;
  char buffer[MAX_PATH];
  DWORD len = MAX_PATH;
  if (((RegOpenKeyExA(HKEY_LOCAL_MACHINE, "Software\\Zusi3", 0, KEY_READ | KEY_WOW64_64KEY, &key) == ERROR_SUCCESS) ||
      (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "Software\\Zusi3", 0, KEY_READ | KEY_WOW64_32KEY, &key) == ERROR_SUCCESS)) &&
       (RegGetValueA(key, nullptr, "DatenVerzeichnis", RRF_RT_REG_SZ, nullptr, (LPBYTE)buffer, &len) == ERROR_SUCCESS ||
        RegGetValueA(key, nullptr, "DatenVerzeichnisSteam", RRF_RT_REG_SZ, nullptr, (LPBYTE)buffer, &len) == ERROR_SUCCESS)) {
    result = std::string(buffer, len - 1);  // buffer ist nullterminiert
    RegCloseKey(key);
  }
#else
  const char* const datapath = getenv("ZUSI3_DATAPATH");
  if (datapath != nullptr) {
    result = std::string(datapath);
  }
#endif
  return result;
}

static inline std::string bestimmeZusiDatenpfadOffiziell() {
  std::string result;
#ifdef _WIN32
  HKEY key;
  char buffer[MAX_PATH];
  DWORD len = MAX_PATH;
  if (((RegOpenKeyExA(HKEY_LOCAL_MACHINE, "Software\\Zusi3", 0, KEY_READ | KEY_WOW64_64KEY, &key) == ERROR_SUCCESS) ||
      (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "Software\\Zusi3", 0, KEY_READ | KEY_WOW64_32KEY, &key) == ERROR_SUCCESS)) &&
       (RegGetValueA(key, nullptr, "DatenVerzeichnisOffiziell", RRF_RT_REG_SZ, nullptr, (LPBYTE)buffer, &len) == ERROR_SUCCESS ||
        RegGetValueA(key, nullptr, "DatenVerzeichnisOffiziellSteam", RRF_RT_REG_SZ, nullptr, (LPBYTE)buffer, &len) == ERROR_SUCCESS)) {
    result = std::string(buffer, len - 1);  // buffer ist nullterminiert
    RegCloseKey(key);
  }
#else
  const char* datapath = getenv("ZUSI3_DATAPATH_OFFICIAL");
  if (datapath == nullptr) {
    datapath = getenv("ZUSI3_DATAPATH");
  }
  if (datapath != nullptr) {
    result = std::string(datapath);
  }
#endif
  return result;
}

inline const std::string& getZusiDatenpfad() {
  static const std::string zusiDatenpfad = bestimmeZusiDatenpfad();
  return zusiDatenpfad;
}

inline const std::string& getZusiDatenpfadOffiziell() {
  static const std::string zusiDatenpfadOffiziell = bestimmeZusiDatenpfadOffiziell();
  return zusiDatenpfadOffiziell;
}

constexpr char zusiSep = '\\';
#ifdef _WIN32
constexpr char osSep = '\\';
#else
constexpr char osSep = '/';
#endif

class ZusiPfad {
public:
  ZusiPfad(const ZusiPfad&) = default;
  ZusiPfad& operator=(const ZusiPfad&) = default;
  ZusiPfad(ZusiPfad&&) = default;
  ZusiPfad& operator=(ZusiPfad&&) = default;

  static ZusiPfad vonZusiPfad(std::string_view zusiPfad, const ZusiPfad& uebergeordnet) {
    if (zusiPfad.empty() || (zusiPfad.find('\\') != std::string_view::npos)) {
      return vonZusiPfad(zusiPfad);
    }

    const auto lastBackslashPos = uebergeordnet.m_pfad.find_last_of('\\');
    if (lastBackslashPos == std::string::npos) {
      return ZusiPfad(std::string(zusiPfad));
    } else {
      std::string pfad = uebergeordnet.m_pfad.substr(0, lastBackslashPos + 1);
      pfad += zusiPfad;
      return ZusiPfad(std::move(pfad));
    }
  }

  static ZusiPfad vonZusiPfad(std::string_view zusiPfad) {
    if (zusiPfad.size() >= 1 && zusiPfad.front() == '\\') {
      return ZusiPfad(std::string(zusiPfad.substr(1)));
    } else {
      return ZusiPfad(std::string(zusiPfad));
    }
  }

  static ZusiPfad vonOsPfad(std::string_view osPfad) {
    std::string relPfad = [&osPfad] {
      // Optimierung fuer (haeufige) einfache Faelle, bei denen ein Stringvergleich genuegt
      const std::string& datenpfadOffiziell = getZusiDatenpfadOffiziell();
      if (std::mismatch(datenpfadOffiziell.begin(), datenpfadOffiziell.end(), osPfad.begin(), osPfad.end()).first == datenpfadOffiziell.end()) {
        return std::string(osPfad.substr(datenpfadOffiziell.size()));
      }

      const std::string& datenpfad = getZusiDatenpfad();
      if (std::mismatch(datenpfad.begin(), datenpfad.end(), osPfad.begin(), osPfad.end()).first == datenpfad.end()) {
        return std::string(osPfad.substr(datenpfad.size()));
      }

      // Allgemeiner Fall
      const auto& lexically_proximate = [](const fs::path& p, const fs::path& base) -> fs::path {
        // Taken and adapted from boost::filesystem, src/path.cpp
        // Licensed under the Boost Software License.
        auto mm = std::mismatch(p.begin(), p.end(), base.begin(), base.end()
#ifdef _WIN32
            , [](const fs::path& lhs, const fs::path& rhs) {
              return strcmpi(lhs.string().c_str(), rhs.string().c_str()) == 0;
            }
#endif
        );
        if ((mm.first == p.begin()) && (mm.second == base.begin())) {
          return p;
        }
        if ((mm.first == p.end()) && (mm.second == base.end())) {
          return ".";
        }
        fs::path tmp;
        while (mm.second != base.end()) {
          tmp /= "..";
          ++mm.second;
        }
        while (mm.first != p.end()) {
          tmp /= *mm.first;
          ++mm.first;
        }
        return tmp;
      };

      const auto& proximate = [&lexically_proximate](const fs::path& p, const fs::path& base) -> fs::path {
#ifdef ZUSI_PARSER_USE_BOOST_FILESYSTEM
        boost::system::error_code ec;
#else
        std::error_code ec;
#endif
        const auto& pCanonical = fs::weakly_canonical(p, ec);
        if (ec) {
          return p;
        }
        const auto& baseCanonical = fs::weakly_canonical(base, ec);
        if (ec) {
          return p;
        }
        return lexically_proximate(pCanonical, baseCanonical);
      };

      const auto relToOffiziell = proximate(
          fs::path(osPfad.begin(), osPfad.end()),
          fs::path(datenpfadOffiziell.begin(), (!datenpfadOffiziell.empty() && datenpfadOffiziell.back() == osSep) ? std::prev(datenpfadOffiziell.end()) : datenpfadOffiziell.end()));
      if (relToOffiziell.is_relative() && (relToOffiziell.empty() || *relToOffiziell.begin() != "..")) {
        return relToOffiziell.string();
      }

      return proximate(
          fs::path(osPfad.begin(), osPfad.end()),
          fs::path(datenpfad.begin(), (!datenpfad.empty() && datenpfad.back() == osSep) ? std::prev(datenpfad.end()) : datenpfad.end())).string();
    }();

    std::replace(relPfad.begin(), relPfad.end(), '/', '\\');
    if (relPfad.back() == '.') {
      relPfad.erase(relPfad.size() - 1);
    }
    return ZusiPfad(std::move(relPfad));
  }

  std::string_view alsZusiPfad() const {
    return m_pfad;
  }

  std::string alsOsPfad() const {
    // Pruefe, ob in eigenem Datenverzeichnis existiert.
    // Wenn nein -> gib Pfad in offiziellem Datenverzeichnis zurueck (unabhaengig davon, ob er existiert)
    std::string resultEigenes = getZusiDatenpfad();
    if constexpr (osSep != zusiSep) {
      std::replace_copy(m_pfad.begin(), m_pfad.end(), std::back_inserter(resultEigenes), zusiSep, osSep);
    } else {
      resultEigenes += m_pfad;
    }

#ifdef _WIN32
    WIN32_FIND_DATAW findData;
#endif

    if (
#ifdef _WIN32
      FindFirstFileW(boost::nowide::widen(resultEigenes).c_str(), &findData) != INVALID_HANDLE_VALUE
#else
      euidaccess(resultEigenes.c_str(), F_OK) == 0
#endif
      ) {
      return resultEigenes;
    }

    std::string resultOffiziell = getZusiDatenpfadOffiziell();
    if constexpr (osSep != zusiSep) {
      std::replace_copy(m_pfad.begin(), m_pfad.end(), std::back_inserter(resultOffiziell), zusiSep, osSep);
    } else {
      resultOffiziell += m_pfad;
    }

    return resultOffiziell;
  }
private:
  // Pfad relativ zum Zusi-Datenverzeichnis; Separator: Backslash; kein fuehrender Backslash
  std::string m_pfad;

  explicit ZusiPfad(std::string pfad) : m_pfad(std::move(pfad)) {}
};

}  // namespace zusixml

#endif  // ZUSI_PARSER_UTILS_HPP_
