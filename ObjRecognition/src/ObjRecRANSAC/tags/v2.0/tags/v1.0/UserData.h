#ifndef USERDATA_H_
#define USERDATA_H_

#include <cstdio>

class UserData
{
public:
	UserData(const char* label = NULL);
	virtual ~UserData();

	const char* getLabel(){ return mLabel;}
	void setLabel(const char* label){ sprintf(mLabel, "%s", label);}

protected:
	char mLabel[2048];
};

#endif /*USERDATA_H_*/
