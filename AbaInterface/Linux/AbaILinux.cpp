#include "MbaInterface.h"

extern "C"
{
	void unload();
}

__attribute__((destructor)) void shared_library_unload(void)
{
	unload();
}
