#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>
#include <cstring>
#include "core/Version.h"

TEST_CASE("cosmos_core links and runs headless") {
    CHECK(std::strlen(cosmos::coreVersion()) > 0);
}
