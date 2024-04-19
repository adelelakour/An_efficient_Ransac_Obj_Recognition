#include "UserData.h"
#include <cstdio>

UserData::UserData(const char* label)
{
	if ( label )
		sprintf(mLabel, "%s", label);
	else
		sprintf(mLabel, "no label");
}

UserData::~UserData()
{
}
