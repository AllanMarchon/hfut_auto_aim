// ROS-free shim for rm_utils/url_resolver.hpp. Resolves `package://<pkg>/<rel>`
// URLs to a filesystem path. With no ament index, the package name maps to a
// compile-time base dir (HFUT_PIPELINE_ASSET_DIR, the pipeline source dir which
// holds config/) or the env override HFUT_PIPELINE_ASSET_DIR. Non-package URLs
// pass through unchanged.
#ifndef RM_UTILS_URL_RESOLVER_HPP_
#define RM_UTILS_URL_RESOLVER_HPP_

#include <cstdlib>
#include <filesystem>
#include <string>

namespace fyt::utils {

class URLResolver {
 public:
  static std::filesystem::path getResolvedPath(const std::string& url) {
    constexpr const char* kPkgPrefix = "package://";
    if (url.rfind(kPkgPrefix, 0) != 0) {
      return std::filesystem::path(url);
    }
    const std::string rest = url.substr(std::string(kPkgPrefix).size());
    const auto slash = rest.find('/');
    const std::string rel = (slash == std::string::npos) ? std::string() : rest.substr(slash + 1);

    std::string base;
    if (const char* env = std::getenv("HFUT_PIPELINE_ASSET_DIR");
        env != nullptr && env[0] != '\0') {
      base = env;
    } else {
#ifdef HFUT_PIPELINE_ASSET_DIR
#define HFUT_RES_STR2(x) #x
#define HFUT_RES_STR(x) HFUT_RES_STR2(x)
      base = HFUT_RES_STR(HFUT_PIPELINE_ASSET_DIR);
#undef HFUT_RES_STR
#undef HFUT_RES_STR2
#endif
    }
    return std::filesystem::path(base) / rel;
  }

 private:
  URLResolver() = delete;
};

}  // namespace fyt::utils

#endif  // RM_UTILS_URL_RESOLVER_HPP_
