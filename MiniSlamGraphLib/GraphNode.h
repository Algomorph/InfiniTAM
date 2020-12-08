// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>

namespace MiniSlamGraph
{
	class GraphNode
	{
	public:
		GraphNode() { mFixed = false; mId = -1; }
		GraphNode(const GraphNode & src) { mFixed = src.mFixed; mId = src.mId; }
		virtual ~GraphNode() {}

		virtual GraphNode* clone() const = 0;
		virtual void applyDelta(const double *delta, const GraphNode *startingPoint = nullptr) = 0;

		virtual int numParameters() const = 0;

		virtual void setParameters(const double *v) = 0;
		virtual void getParameters(double *v) = 0;

		bool isFixed() const { return mFixed; }
		void setFixed(bool value = true) { mFixed = value; }

		int getId() const { return mId; }
		void setId(int id) { mId = id; }

	private:
		bool mFixed;
		int mId;
	};
}
