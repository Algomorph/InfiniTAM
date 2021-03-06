// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "GraphNode.h"

#include "../ORUtils/SE3Pose.h"

namespace MiniSlamGraph
{
	class GraphNodeSE3 : public GraphNode
	{
	public:
		typedef ORUtils::SE3Pose SE3;
		GraphNodeSE3() {}
		GraphNodeSE3(const GraphNodeSE3 & src) : GraphNode(src) { mPose.SetFrom(&(src.mPose)); }
		GraphNodeSE3(const SE3 & pose) : mPose(pose) {}

		GraphNodeSE3* clone() const
		{
			return new GraphNodeSE3(*this);
		}

		void applyDelta(const double *delta, const GraphNode *startingPoint = nullptr)
		{
			SE3 startingPose;
			if (startingPoint == nullptr) startingPose.SetFrom(&mPose);
			else startingPose.SetFrom(&(((const GraphNodeSE3*)startingPoint)->mPose));

			SE3 deltaPose((float)delta[0], (float)delta[1], (float)delta[2], (float)delta[3], (float)delta[4], (float)delta[5]);
			mPose.SetM(deltaPose.GetM() * startingPose.GetM());
		}

		int numParameters() const { return 6; }

		void setParameters(const double *v)
		{
			mPose = SE3((float)v[0], (float)v[1], (float)v[2], (float)v[3], (float)v[4], (float)v[5]);
		}

		void getParameters(double *v)
		{
			for (int i = 0; i < 6; ++i) v[i] = mPose.GetParams()[i];
		}

		const SE3 & getPose() const
		{
			return mPose;
		}

		void setPose(const SE3 & pose)
		{
			mPose.SetFrom(&pose);
		}

	private:
		SE3 mPose;
	};
}