#pragma once

#if defined(TARGET_STANDARD)
    #include "standard.hpp"
#elif defined(TARGET_HERO)
    #include "hero.hpp"
#elif defined(TARGET_SENTRY)
    #include "sentry.hpp"
#else
    #error "robot type not supported"
#endif