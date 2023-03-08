#pragma once

#include <unordered_map>
#include <string>

#include <windows.h>
#include <mmsystem.h>
#include <playsoundapi.h>
#pragma comment(lib, "Winmm.lib")

extern std::unordered_map<std::string, std::string> soundMap;

class SoundHandler {

public:
	static SoundHandler& getInstance() {
		static SoundHandler instace;
		return instace;
	}
	void PlayOurSound(std::string soundName, bool loop = false);

private:
	SoundHandler() {}

};