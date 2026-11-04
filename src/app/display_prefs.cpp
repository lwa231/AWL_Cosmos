#include "app/display_prefs.h"

namespace cosmos {
namespace app {

DisplayPrefs& displayPrefs()
{
    static DisplayPrefs prefs;
    return prefs;
}

} // namespace app
} // namespace cosmos
