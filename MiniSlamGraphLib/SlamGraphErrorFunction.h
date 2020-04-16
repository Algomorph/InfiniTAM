// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "MatrixWrapper.h"
#include "SlamGraph.h"

namespace MiniSlamGraph {

	class SlamGraphErrorFunction /*: public K_OPTIM::ErrorFunctionLeastSquares*/ 
	{
	public:
		class Parameters //: public K_OPTIM::OptimizationParameter
		{
		public:
			Parameters(const SlamGraph & graph);
			Parameters(const Parameters & src);
			~Parameters();
			Parameters* clone() const { return new Parameters(*this); }
			void copyFrom(const /*Optimization*/Parameters & _src);
			//void copyValuesFrom(const OptimizationParameter & src);
			void clear();

			const SlamGraph::NodeIndex & getNodes() const
			{
				return mNodes;
			}
			SlamGraph::NodeIndex & getNodes()
			{
				return mNodes;
			}

		private:
			SlamGraph::NodeIndex mNodes;
		};

		class EvaluationPoint /*: public K_OPTIM::ErrorFunctionLeastSquares::EvaluationPoint*/ 
		{
		public:
			EvaluationPoint(const SlamGraphErrorFunction *parent, Parameters *para);
			~EvaluationPoint();

			double f();
			const double* nabla_f();
			const Matrix* hessian_GN();
			const Parameters & getParameter() const { return *mPara; }

		private:
			void cacheGH();

			const SlamGraphErrorFunction *mParent;
			const Parameters *mPara;
			double cacheF;
			VariableLengthVector *cacheG;
			Matrix *cacheH;
		};

		SlamGraphErrorFunction(const SlamGraph & graph);

		~SlamGraphErrorFunction();

		int numParameters() const;

		EvaluationPoint* evaluateAt(/*K_OPTIM::Optimization*/Parameters *para) const;

		void applyDelta(const /*K_OPTIM::Optimization*/Parameters & para_old, const double *delta, /*K_OPTIM::Optimization*/Parameters & para_new) const;

		//Matrix_CSparse::Pattern* & getHessianSparsityPattern()
		void* & getHessianSparsityPattern()
		{
			return mSparsityPattern;
		}

		const SlamGraph* getGraph() const
		{
			return mGraph;
		}

	private:
		const SlamGraph *mGraph;
		void *mSparsityPattern;
	};
}

