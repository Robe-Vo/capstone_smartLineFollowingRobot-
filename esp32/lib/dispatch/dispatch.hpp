#pragma once
#include <stdint.h>
#include "protocol.hpp"

/**     ====== DISPATCH ====== 
 * 
 *  This library assign each cmd to each action
 */

namespace Dispatch {

    using Handler = void(*)(const Protocol::RxFrame&);

    void init();                              // register handlers
    bool run(const Protocol::RxFrame& f);     // execute handler by cmd

}
