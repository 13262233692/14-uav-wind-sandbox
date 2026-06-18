#include "UAVWindSandboxGameModeBase.h"
#include "Actors/SolarUAV.h"

AUAVWindSandboxGameModeBase::AUAVWindSandboxGameModeBase()
{
	DefaultPawnClass = ASolarUAV::StaticClass();
}
