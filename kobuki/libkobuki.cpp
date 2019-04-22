#include "libkobuki.h"
#include "Kobuki_impl.h"

LIBKOBUKI_API rt_net::Kobuki* createKobuki(const rt_net::KobukiArgument &arg)
{
  return new rt_net::Kobuki_impl(*(dynamic_cast<const rt_net::KobukiStringArgument*>(&arg)));
}
