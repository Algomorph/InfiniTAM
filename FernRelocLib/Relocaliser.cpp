// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "Relocaliser.h"
#include "PixelUtils.h"

#include <iostream>
#include <fstream>

using namespace FernRelocLib;

Relocaliser::Relocaliser(ORUtils::Vector2<int> imgSize, ORUtils::Vector2<float> range, float harvestingThreshold, int numFerns, int numDecisionsPerFern)
{
	static const int levels = 5;
	encoding = new FernConservatory(numFerns, imgSize / (1 << levels), range, numDecisionsPerFern);
	relocDatabase = new RelocDatabase(numFerns, encoding->getNumCodes());
	poseDatabase = new PoseDatabase();
	keyframeHarvestingThreshold = harvestingThreshold;

	processedImage1 = new ORUtils::Image<float>(imgSize, MEMORYDEVICE_CPU);
	processedImage2 = new ORUtils::Image<float>(imgSize, MEMORYDEVICE_CPU);
}

Relocaliser::~Relocaliser()
{
	delete encoding;
	delete relocDatabase;
	delete poseDatabase;
	delete processedImage1;
	delete processedImage2;
}

int Relocaliser::ProcessFrame(const ORUtils::Image<float> *img, const ORUtils::SE3Pose *pose, int sceneId, int k, int nearestNeighbours[], float *distances, bool harvestKeyframes) const
{
	// downsample and preprocess image => processedImage1
	filterSubsample(img, processedImage1); // 320x240
	filterSubsample(processedImage1, processedImage2); // 160x120
	filterSubsample(processedImage2, processedImage1); // 80x60
	filterSubsample(processedImage1, processedImage2); // 40x30

	filterGaussian(processedImage2, processedImage1, 2.5f);

	// compute code
	int codeLength = encoding->getNumFerns();
	char *code = new char[codeLength];
	encoding->computeCode(processedImage1, code);

	// prepare outputs
	int ret = -1;
	bool releaseDistances = (distances == NULL);
	if (distances == NULL) distances = new float[k];

	// find similar frames
	int similarFound = relocDatabase->findMostSimilar(code, nearestNeighbours, distances, k);

	// add keyframe to database
	if (harvestKeyframes)
	{
		if (similarFound == 0) ret = relocDatabase->addEntry(code);
		else if (distances[0] > keyframeHarvestingThreshold) ret = relocDatabase->addEntry(code);

		if (ret >= 0) poseDatabase->storePose(ret, *pose, sceneId);
	}

	// cleanup and return
	delete[] code;
	if (releaseDistances) delete[] distances;
	return ret;
}

const FernRelocLib::PoseDatabase::PoseInScene & Relocaliser::RetrievePose(int id)
{
	return poseDatabase->retrievePose(id);
}

void Relocaliser::SaveToDirectory(const std::string& outputDirectory)
{
	std::string configFilePath = outputDirectory + "config.txt";
	std::ofstream ofs(configFilePath.c_str());

	if (!ofs) throw std::runtime_error("Could not open " + configFilePath + " for reading");
	ofs << "type=rgb,levels=4,numFerns=" << encoding->getNumFerns() << ",numDecisionsPerFern=" << encoding->getNumDecisions() / 3 << ",harvestingThreshold=" << keyframeHarvestingThreshold;

	encoding->SaveToFile(outputDirectory + "ferns.txt");
	relocDatabase->SaveToFile(outputDirectory + "frames.txt");
	poseDatabase->SaveToFile(outputDirectory + "poses.txt");
}

void Relocaliser::LoadFromDirectory(const std::string& inputDirectory)
{
	std::string fernFilePath = inputDirectory + "ferns.txt";
	std::string frameCodeFilePath = inputDirectory + "frames.txt";
	std::string posesFilePath = inputDirectory + "poses.txt";

	if (!std::ifstream(fernFilePath.c_str()))
		throw std::runtime_error("unable to open " + fernFilePath);
	if (!std::ifstream(frameCodeFilePath.c_str()))
		throw std::runtime_error("unable to open " + frameCodeFilePath);
	if (!std::ifstream(posesFilePath.c_str()))
		throw std::runtime_error("unable to open " + posesFilePath);

	encoding->LoadFromFile(fernFilePath);
	relocDatabase->LoadFromFile(frameCodeFilePath);
	poseDatabase->LoadFromFile(posesFilePath);
}