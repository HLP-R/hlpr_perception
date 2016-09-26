/*
 * ircpHandler.hpp
 *
 *  Created on: Dec 27, 2012
 *      Author: siml
 */

#ifndef IRCPHANDLER_HPP_
#define IRCPHANDLER_HPP_

#include <string>
#include <ircp/ircp.h>
#include "objectFeatures.hpp"
#include <fstream>
#include <iostream>

using namespace IRCP;

//#define USE_INDEX_ARRAY

enum SpeechCommandType {
	NONE = 0,
	RECORD_KEYFRAME = 1,
	NEW_DEMO = 2,
	END_DEMO = 3,
	OPEN_HAND = 4,
	CLOSE_HAND = 5,
	START_TRAJECTORY =6,
	END_TRAJECTORY =7,
	NEW_SKILL = 8
};

//precision grasp?
#define NUM_SPEECH_COMMANDS 8

struct serviceRequest;
void resetServiceRequest(serviceRequest &request);

struct serviceRequest {
	bool recordFrame;
	bool newDemo;
	bool endDemo;
	bool bgTrain;
	bool startLoop;
	bool stopLoop;
	bool quit;
	bool isViz;
	bool newSkill;
	bool isObservationOnly;
	bool justTra;
	bool justKF;
	bool noHand;
	SpeechCommandType speechCommand;
	serviceRequest (){resetServiceRequest(*this);} //: recordFrame(false), newDemo(false), bgTrain(false), startLoop(false), stopLoop(false), quit(false), isViz(true), speechCommand(NONE)  {}
};

class skillInfo {
private:
	char tmp[100];

public:
	int kfNum;
	int demoNum;
	int obsDemoNum;
	int subjectNum;
	int hueValue;
	std::string skillName;
	std::string dataLocation;
	std::string rogueSkillName;
	char rogueSkillCounter;
	bool newInfo;
	bool isObservationOnly;
	bool firstInfo;

	skillInfo () : kfNum(0), demoNum(0), obsDemoNum(0), subjectNum(0), skillName("skill"), newInfo(false),
			dataLocation("/home/baris/data/agLearning/sim"), isObservationOnly(false), firstInfo(true),
	                //dataLocation("/home/siml/data/tesca"), isObservationOnly(true), //
			rogueSkillName("skill"), rogueSkillCounter('0'), hueValue(-1) {}

	void getDataFolder(char *folderName) {
		sprintf(folderName,"%s", dataLocation.c_str());
	}
	void getSubjectFolder(char *folderName) {
		getDataFolder(tmp);
		sprintf(folderName,"%s/subject%d", tmp, subjectNum);
	}
	void getSkillFolder(char *folderName) {
		getSubjectFolder(tmp);
		sprintf(folderName,"%s/%s", tmp, skillName.c_str());
	}

	void getFileName(char *imageName) {
		getSkillFolder(tmp);
		if(isObservationOnly)
			sprintf(imageName,"%s/observation_demo%d_kf%d", tmp, obsDemoNum, kfNum);
		else
			sprintf(imageName,"%s/demo%d_kf%d", tmp, demoNum, kfNum);
	}

	void getFileNameWithExt(char *imageName, const char *ext) {
		getFileName(tmp);
		sprintf(imageName,"%s.%s", tmp, ext);
	}
};

void resetServiceRequest(serviceRequest &request) {
	request.recordFrame = false;
	request.newDemo = false;
	request.endDemo = false;
	request.bgTrain = false;
	request.startLoop = false;
	request.stopLoop = false;
	request.quit = false;
	request.isViz = true;
	request.newSkill = false;
	request.speechCommand = NONE;
	request.isObservationOnly = false;
	request.justTra = false;
	request.justKF  = !request.justTra && true;
	request.noHand  = true;
	return;
}

struct commands4Loop {
        bool loop_break;
        bool loop_continue;
        bool save_raw;
        bool save_features;
        bool send_featurs;
        bool send_misc_info;
        bool make_skill_folder;
        commands4Loop() : loop_break(false), loop_continue(false), save_raw(false),
                                  save_features(false), send_featurs(false), send_misc_info(false),
                                  make_skill_folder(false){}
};

//serviceRequest globalRequests;

//namespace IRCP_HANDLER {

const char* _bcast = "255.255.255.255";

//Move these to ircp.h when they are finalized enough
#define OVERHEAD_CAM_MODULE_GENERIC_INFO_REQUEST 201
#define SKILL_INFO 202
#define OVERHEAD_OBJECT_DATA 203
typedef BasicSubpacket <VISION_MAJOR_TYPE, OVERHEAD_CAM_MODULE_GENERIC_INFO_REQUEST, Integer> GenericInfoRequest;
typedef ArraySubpacket <VISION_MAJOR_TYPE, SKILL_INFO, IRCP::String> InfoData;

#ifdef USE_INDEX_ARRAY
typedef IndexedArraySubpacket <VISION_MAJOR_TYPE, OVERHEAD_OBJECT_DATA, IRCP::Float> ObjectData;
#else
typedef ArraySubpacket <VISION_MAJOR_TYPE, OVERHEAD_OBJECT_DATA, IRCP::Float> ObjectData;
#endif



void speechCommandCallback(unsigned char src,  SpeechCommandData::iterator begin,	SpeechCommandData::iterator end);

//background training, reset training, start, stop sending, etc simple commands
//current design is using a float to distinguish between commands, might change in the future
void genericRequestCallback(unsigned char src,  const IRCP::Float& val);

//info sent from c6m. what should be the format? Decided on indexed array of string
void informationCallback(unsigned char src, InfoData::iterator begin, InfoData::iterator end);

//convenience function
void mapSpeechToBools();


//}
void fillObjectInfo(std::vector<pc_cluster_features> &objFeatures);
void fillObjectInfo(pc_cluster_features &objFeature);

//IRCP::IndexedArray<Float> globalObjectInfo; //sorry to do this globally, in a hurry
//if you are using array instead of indewxed array, you should just create the array yourself and give it the start and end pointers of the array
float objectInfoBuffer[5000];

class ircpHandler {

private:
	unsigned char _robot;
	unsigned char _module; //this module
	unsigned char _targetModule; //targetModules in the future?

	bool idleMode;
	char charBuff[100];

	bool _registerCallbacks;

	Module *ircp;

	//IRCP Receivers Handlers were not written as methods to keep things simple (it is a pain to get pointers to class methods, at least not easily understandable for maintanence)

	//friend void speechCommandCallback(unsigned char src,  SpeechCommandData::iterator begin,	SpeechCommandData::iterator end);


public:
	ircpHandler(unsigned char targetModule = BEHAVIOR_MODULE_ID, unsigned char module = OVERHEAD_VISION_BASLER_MODULE_ID, unsigned char robot = SIMON_ID, bool registerCallbacks = true);
	~ircpHandler();

	void initialize();

	static serviceRequest globalRequests;
	static skillInfo globalSkillInfo;



	//IRCP Senders
	void sendSceneInformation();  //just the high level stuff, only the number of objects right now
	void sendObjectInformation(); //Information about detected objects

	commands4Loop handleCommands(char *imageName);

	Module *getIrcpHandle() {return ircp;}

};

#endif /* IRCPHANDLER_HPP_ */

