#include "SoundHandler.h"

std::unordered_map<std::string, std::string> soundMap = {
	{"eat", "data/eat.wav"},
	{"gameover", "data/gameover.wav"},
	{"pickup", "data/pickup.wav"},
	{"button", "data/button.wav"},
	{"music", "data/musicut.wav"}
};

void SoundHandler::PlayOurSound(std::string soundName, bool loop) {
	if (!loop) PlaySound(soundMap[soundName].c_str(), NULL, SND_FILENAME | SND_ASYNC); // SND_ASYNC is for performance
	else PlaySound(soundMap[soundName].c_str(), NULL, SND_FILENAME | SND_ASYNC | SND_LOOP);
}
