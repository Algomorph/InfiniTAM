//  ================================================================
//  Created by Gregory Kramida on 6/28/18.
//  Copyright (c) 2018-2025 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================
#pragma once

//VTK
#include <vtkInteractorStyleTrackballCamera.h>

namespace ITMLib{
template<typename TVoxelCanonical,typename TVoxelLive,typename TIndex>
class ITMSceneSliceVisualizer3DInteractorStyle : public vtkInteractorStyleTrackballCamera {
public:
	static ITMSceneSliceVisualizer3DInteractorStyle* New();
	vtkTypeMacro(ITMSceneSliceVisualizer3DInteractorStyle , vtkInteractorStyleTrackballCamera);
	ITMSceneSliceVisualizer3DInteractorStyle();
	void SetParent(ITMSceneSliceVisualizer3D<TVoxelCanonical,TVoxelLive,TIndex>* parent);

	void OnKeyPress() override;
private:

	ITMSceneSliceVisualizer3D<TVoxelCanonical,TVoxelLive,TIndex>* parent;
	int keyBindingLayerIndex;
	bool bindingLayerShowing = false;

	typedef void (ITMSceneSliceVisualizer3DInteractorStyle<TVoxelCanonical,TVoxelLive,TIndex>::*MFP) ();
	struct UserAction{
		std::string description;
		MFP function;
	};

	void PrintMIAU();
	void PrintHaha();
	void ToggleKeyBindingOverlay();
	void RequestShutdown();
	void SetVisibilityToCanonicalWithUpdates();
	void SetVisibilityToLive();
	void SetVisibilityToLiveAndCanonicalWithUpdates();
	void SetVisibilityToFusedCanonical();
	void AdvanceVisibleOptimizationStep();
	void RetreatVisibleOptimizationStep();


	std::map<std::string, UserAction> createBindingMap();
	const std::map<std::string, UserAction> bindingMap;

};
}


