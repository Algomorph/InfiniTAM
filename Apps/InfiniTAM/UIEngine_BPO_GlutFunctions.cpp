//  ================================================================
//  Created by Gregory Kramida on 5/15/18.
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
#include "UIEngine_BPO.h"
#include "../../ITMLib/Utils/Analytics/BenchmarkUtilities.h"
#include "../../ITMLib/Engines/Main/MultiEngine.h"
#include "../../ITMLib/Engines/Main/BasicVoxelEngine.h"
#include "../../ITMLib/Utils/Telemetry/TelemetryRecorder.h"


//TODO: we should never have to downcast the main engine to some other engine type, architecture needs to be altered
// (potentially by introducting empty method stubs) -Greg (GitHub:Algomorph)


#ifdef __APPLE__
#include <GLUT/glut.h>
#else

#include <GL/glut.h>

#endif

#ifdef FREEGLUT

#include <GL/freeglut.h>

#else
#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "glut64")
#endif
#endif

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;


static void Safe_GlutBitmapString(void* font, const char* str) {
	size_t len = strlen(str);
	for (size_t x = 0; x < len; ++x) {
		glutBitmapCharacter(font, str[x]);
	}
}

void UIEngine_BPO::GlutDisplayFunction() {
	UIEngine_BPO& uiEngine = UIEngine_BPO::Instance();

	// get updated images from processing thread
	uiEngine.mainEngine->GetImage(uiEngine.outImage[0], uiEngine.outImageType[0], &uiEngine.freeviewPose,
	                              &uiEngine.freeviewIntrinsics);

	for (int w = 1; w < NUM_WIN; w++) {
		uiEngine.mainEngine->GetImage(uiEngine.outImage[w], uiEngine.outImageType[w]);
	}

	// do the actual drawing
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(1.0f, 1.0f, 1.0f);
	glEnable(GL_TEXTURE_2D);

	ITMUChar4Image** showImgs = uiEngine.outImage;
	Vector4f* winReg = uiEngine.winReg;
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	{
		glLoadIdentity();
		glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		{
			glEnable(GL_TEXTURE_2D);
			for (int w = 0; w < NUM_WIN; w++) {// Draw each sub window
				if (uiEngine.outImageType[w] == MainEngine::InfiniTAM_IMAGE_UNKNOWN) continue;
				glBindTexture(GL_TEXTURE_2D, uiEngine.textureId[w]);
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, showImgs[w]->dimensions.x, showImgs[w]->dimensions.y, 0, GL_RGBA,
				             GL_UNSIGNED_BYTE, showImgs[w]->GetData(MEMORYDEVICE_CPU));
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
				glBegin(GL_QUADS);
				{
					glTexCoord2f(0, 1);
					glVertex2f(winReg[w][0], winReg[w][1]); // glVertex2f(0, 0);
					glTexCoord2f(1, 1);
					glVertex2f(winReg[w][2], winReg[w][1]); // glVertex2f(1, 0);
					glTexCoord2f(1, 0);
					glVertex2f(winReg[w][2], winReg[w][3]); // glVertex2f(1, 1);
					glTexCoord2f(0, 0);
					glVertex2f(winReg[w][0], winReg[w][3]); // glVertex2f(0, 1);
				}
				glEnd();
			}
			glDisable(GL_TEXTURE_2D);
		}
		glPopMatrix();
	}
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	switch (uiEngine.trackingResult) {
		case ITMLib::CameraTrackingState::TrackingResult::TRACKING_FAILED:
			glColor3f(1.0f, 0.0f, 0.0f);
			break; // failure
		case ITMLib::CameraTrackingState::TrackingResult::TRACKING_POOR:
			glColor3f(1.0f, 1.0f, 0.0f);
			break; // poor
		case ITMLib::CameraTrackingState::TrackingResult::TRACKING_GOOD:
			glColor3f(0.0f, 1.0f, 0.0f);
			break; // good
		default:
			//TODO: why isn't this a separate value in the TrackingResult enum?
			glColor3f(1.0f, 1.0f, 1.0f);
			break; // relocalising
	}

	char str[200];

	//print previous frame index
	int lastFrameIx = uiEngine.start_frame_index + uiEngine.processedFrameNo - 1;
	if (lastFrameIx >= 0) {
		glRasterPos2f(0.775f, -0.900f);
		sprintf(str, "Frame %5d", lastFrameIx);
		Safe_GlutBitmapString(GLUT_BITMAP_HELVETICA_18, (const char*) str);
	}

	//print frame rate
	glRasterPos2f(0.85f, -0.962f);
	sprintf(str, "%04.2lf", uiEngine.processedTime);
	Safe_GlutBitmapString(GLUT_BITMAP_HELVETICA_18, (const char*) str);

	glColor3f(1.0f, 0.0f, 0.0f);

	glRasterPos2f(-0.98f, -0.90f);
	const char* modeName;
	const char* followOrFreeview;
	if (uiEngine.freeview_active) {
		modeName = uiEngine.colourModes_freeview[uiEngine.current_colour_mode].name;
		followOrFreeview = "follow camera";
	} else {
		modeName = uiEngine.colourModes_main[uiEngine.current_colour_mode].name;
		followOrFreeview = "free viewpoint";
	}

	//Draw keyboard shortcut legend
	sprintf(str, "n: one frame \t b: continuous \t q/e/esc: exit \t r: reset \t s: save scene \t l: load scene\t"
	             " f: %s \t c: colours (currently %s) \t t: turn fusion %s", followOrFreeview, modeName,
	        uiEngine.integration_active ? "off" : "on");
	Safe_GlutBitmapString(GLUT_BITMAP_HELVETICA_12, (const char*) str);
	glRasterPos2f(-0.98f, -0.95f);
	sprintf(str,
	        "i: %d frames \t d: one step \t p: pause \t v: write video %s \t ",
	        uiEngine.number_of_frames_to_process_after_launch,
	        uiEngine.depthVideoWriter != nullptr ? "off" : "on");
	Safe_GlutBitmapString(GLUT_BITMAP_HELVETICA_12, (const char*) str);

	glutSwapBuffers();
	uiEngine.needsRefresh = false;
}

void handle_check_end_automatic_run(UIEngine_BPO& engine) {

}

void UIEngine_BPO::GlutIdleFunction() {
	UIEngine_BPO& uiEngine = UIEngine_BPO::Instance();
	if (uiEngine.shutdownRequested) {
		uiEngine.mainLoopAction = UIEngine_BPO::EXIT;
	}
	switch (uiEngine.mainLoopAction) {
		case PROCESS_FRAME:
			uiEngine.ProcessFrame();
			uiEngine.processedFrameNo++; //done with current frame, increment the frame counter
			uiEngine.mainLoopAction = PROCESS_PAUSED;
			uiEngine.needsRefresh = true;
			break;
		case PROCESS_VIDEO:
			uiEngine.ProcessFrame();
			uiEngine.processedFrameNo++;
			uiEngine.needsRefresh = true;
			break;
		case PROCESS_N_FRAMES:
			uiEngine.ProcessFrame();
			uiEngine.processedFrameNo++;
			uiEngine.needsRefresh = true;
			if ((uiEngine.processedFrameNo - uiEngine.auto_interval_frame_start) >=
			    uiEngine.number_of_frames_to_process_after_launch) {
				uiEngine.mainLoopAction = uiEngine.exit_after_automatic_run ? EXIT : PROCESS_PAUSED;
				if (uiEngine.save_after_automatic_run) {
					uiEngine.mainEngine->SaveToFile(uiEngine.GeneratePreviousFrameOutputPath());
				}
				if (configuration::get().telemetry_settings.log_benchmarks) {
					bench::log_all_timers();
				}
			}
			break;
		case EXIT:
#ifdef FREEGLUT
			glutLeaveMainLoop();
#else
			exit(0);
#endif
			break;
		case PROCESS_PAUSED:
		default:
			break;
	}

	if (uiEngine.needsRefresh) {
		glutPostRedisplay();
	}
}

void UIEngine_BPO::GlutKeyUpFunction(unsigned char key, int x, int y) {
	UIEngine_BPO& uiEngine = UIEngine_BPO::Instance();
	int modifiers = glutGetModifiers();

	switch (key) {
		//TODO: rearrange in asciibeditc order (except fall-through cases) to make maintenance easier
		case 'i':
			printf("processing %d frames ...\n", uiEngine.number_of_frames_to_process_after_launch);
			uiEngine.auto_interval_frame_start = uiEngine.processedFrameNo;
			uiEngine.mainLoopAction = UIEngine_BPO::PROCESS_N_FRAMES;
			break;
		case 'b':
			printf("processing input source ...\n");
			uiEngine.mainLoopAction = UIEngine_BPO::PROCESS_VIDEO;
			break;
		case 'n':
			uiEngine.PrintProcessingFrameHeader();
			uiEngine.mainLoopAction = UIEngine_BPO::PROCESS_FRAME;
			break;
		case 'k':
			if (uiEngine.isRecordingImages) {
				printf("stopped recoding to disk ...\n");
				uiEngine.isRecordingImages = false;
			} else {
				printf("started recoding to disk ...\n");
				uiEngine.processed_frame_count = 0;
				uiEngine.isRecordingImages = true;
			}
			break;
		case 'v':
			if (modifiers & GLUT_ACTIVE_ALT) {
				if ((uiEngine.reconstructionVideoWriter != nullptr)) {
					printf("stopped recoding reconstruction video\n");
					delete uiEngine.reconstructionVideoWriter;
					uiEngine.reconstructionVideoWriter = nullptr;
				} else {
					printf("started recoding reconstruction video\n");
					uiEngine.reconstructionVideoWriter = new FFMPEGWriter();
				}
			} else {
				if ((uiEngine.rgbVideoWriter != nullptr) || (uiEngine.depthVideoWriter != nullptr)) {
					printf("stopped recoding input video\n");
					delete uiEngine.rgbVideoWriter;
					delete uiEngine.depthVideoWriter;
					uiEngine.rgbVideoWriter = nullptr;
					uiEngine.depthVideoWriter = nullptr;
				} else {
					printf("started recoding input video\n");
					uiEngine.rgbVideoWriter = new FFMPEGWriter();
					uiEngine.depthVideoWriter = new FFMPEGWriter();
				}
			}
			break;
		case 'q':
		case 'e':
		case 27: // esc key
			printf("exiting ...\n");
			uiEngine.mainLoopAction = UIEngine_BPO::EXIT;
			break;
		case 'f':
			uiEngine.current_colour_mode = 0;
			//TODO: replace this whole if/else block with a separate function, use this function during initialization as well -Greg (Github: Algomorph)
			if (uiEngine.freeview_active) {
				uiEngine.outImageType[0] = MainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
				uiEngine.outImageType[1] = MainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;

				uiEngine.freeview_active = false;
			} else {
				uiEngine.outImageType[0] = MainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED;
				uiEngine.outImageType[1] = MainEngine::InfiniTAM_IMAGE_SCENERAYCAST;

				uiEngine.freeviewPose.SetFrom(uiEngine.mainEngine->GetTrackingState()->pose_d);
				if (uiEngine.mainEngine->GetView() != nullptr) {
					uiEngine.freeviewIntrinsics = uiEngine.mainEngine->GetView()->calib.intrinsics_d;
					uiEngine.outImage[0]->ChangeDims(uiEngine.mainEngine->GetView()->depth->dimensions);
				}

				switch (uiEngine.indexing_method) {
					case configuration::INDEX_HASH: {
						auto* multiEngine = dynamic_cast<MultiEngine<TSDFVoxel, VoxelBlockHash>*>(uiEngine.mainEngine);
						if (multiEngine != nullptr) {
							int idx = multiEngine->findPrimaryLocalMapIdx();
							if (idx < 0) idx = 0;
							multiEngine->setFreeviewLocalMapIdx(idx);
						}
					}
						break;
					case configuration::INDEX_ARRAY:
						auto* multiEngine = dynamic_cast<MultiEngine<TSDFVoxel, PlainVoxelArray>*>(uiEngine.mainEngine);
						if (multiEngine != nullptr) {
							int idx = multiEngine->findPrimaryLocalMapIdx();
							if (idx < 0) idx = 0;
							multiEngine->setFreeviewLocalMapIdx(idx);
						}
						break;
				}


				uiEngine.freeview_active = true;
			}
			uiEngine.needsRefresh = true;
			break;
		case 'c':
			uiEngine.current_colour_mode++;
			if (((uiEngine.freeview_active) &&
			     ((unsigned) uiEngine.current_colour_mode >= uiEngine.colourModes_freeview.size())) ||
			    ((!uiEngine.freeview_active) &&
			     ((unsigned) uiEngine.current_colour_mode >= uiEngine.colourModes_main.size())))
				uiEngine.current_colour_mode = 0;
			uiEngine.needsRefresh = true;
			break;
		case 't': {
			uiEngine.integration_active = !uiEngine.integration_active;
			uiEngine.mainEngine->turnOffIntegration();
		}
			break;
		case 'r': {
			uiEngine.mainEngine->resetAll();
		}
			break;
		case 's': {
			if (modifiers && GLUT_ACTIVE_ALT) {
				printf("saving scene to model ... ");
				uiEngine.mainEngine->SaveSceneToMesh("mesh.stl");
				printf("done\n");
			} else {
				printf("saving scene to disk ... ");
				try {
					uiEngine.mainEngine->SaveToFile();
					printf("done\n");
				}
				catch (const std::runtime_error& e) {
					printf("failed: %s\n", e.what());
				}
			}
		}
			break;
		case 'l': {
			printf("loading scene from disk ... ");

			try {
				uiEngine.mainEngine->LoadFromFile(uiEngine.GenerateCurrentFrameOutputPath());
				printf("done\n");
			}
			catch (const std::runtime_error& e) {
				printf("failed: %s\n", e.what());
			}
		}
			break;
		case 'p':
			uiEngine.mainLoopAction = PROCESS_PAUSED;
			break;
		case '[':
		case ']': {
			auto* multiEngineVBH = dynamic_cast<MultiEngine<TSDFVoxel, VoxelBlockHash>*>(uiEngine.mainEngine);
			if (multiEngineVBH != nullptr) {
				int idx = multiEngineVBH->getFreeviewLocalMapIdx();
				if (key == '[') idx--;
				else idx++;
				multiEngineVBH->changeFreeviewLocalMapIdx(&(uiEngine.freeviewPose), idx);
				uiEngine.needsRefresh = true;
			}
			auto* multiEnginePVA = dynamic_cast<MultiEngine<TSDFVoxel, PlainVoxelArray>*>(uiEngine.mainEngine);
			if (multiEnginePVA != nullptr) {
				int idx = multiEnginePVA->getFreeviewLocalMapIdx();
				if (key == '[') idx--;
				else idx++;
				multiEnginePVA->changeFreeviewLocalMapIdx(&(uiEngine.freeviewPose), idx);
				uiEngine.needsRefresh = true;
			}
		}
			break;
	}
	if (uiEngine.freeview_active) uiEngine.outImageType[0] = uiEngine.colourModes_freeview[uiEngine.current_colour_mode].type;
	else uiEngine.outImageType[0] = uiEngine.colourModes_main[uiEngine.current_colour_mode].type;
}

void UIEngine_BPO::GlutMouseButtonFunction(int button, int state, int x, int y) {
	UIEngine_BPO& uiEngine = UIEngine_BPO::Instance();

	if (state == GLUT_DOWN) {
		switch (button) {
			case GLUT_LEFT_BUTTON:
				uiEngine.mouseState = 1;
				break;
			case GLUT_MIDDLE_BUTTON:
				uiEngine.mouseState = 3;
				break;
			case GLUT_RIGHT_BUTTON:
				uiEngine.mouseState = 2;
				break;
			default:
				break;
		}
		uiEngine.mouseLastClick.x = x;
		uiEngine.mouseLastClick.y = y;

		glutSetCursor(GLUT_CURSOR_NONE);
	} else if (state == GLUT_UP && !uiEngine.mouseWarped) {
		uiEngine.mouseState = 0;
		glutSetCursor(GLUT_CURSOR_INHERIT);
	}
}

static inline Matrix3f createRotation(const Vector3f& _axis, float angle) {
	Vector3f axis = normalize(_axis);
	float si = sinf(angle);
	float co = cosf(angle);

	Matrix3f ret;
	ret.setIdentity();

	ret *= co;
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) ret.at(c, r) += (1.0f - co) * axis[c] * axis[r];

	Matrix3f skewmat;
	skewmat.setZeros();
	skewmat.at(1, 0) = -axis.z;
	skewmat.at(0, 1) = axis.z;
	skewmat.at(2, 0) = axis.y;
	skewmat.at(0, 2) = -axis.y;
	skewmat.at(2, 1) = axis.x;
	skewmat.at(1, 2) = -axis.x;
	skewmat *= si;
	ret += skewmat;

	return ret;
}

void UIEngine_BPO::GlutMouseMoveFunction(int x, int y) {
	UIEngine_BPO& uiEngine = UIEngine_BPO::Instance();

	if (uiEngine.mouseWarped) {
		uiEngine.mouseWarped = false;
		return;
	}

	if (!uiEngine.freeview_active || uiEngine.mouseState == 0) return;

	Vector2i movement;
	movement.x = x - uiEngine.mouseLastClick.x;
	movement.y = y - uiEngine.mouseLastClick.y;

	Vector2i realWinSize(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
	// Does not work if the window is smaller than 40x40
	Vector2i activeWinTopLeft(20, 20);
	Vector2i activeWinBottomRight(realWinSize.width - 20, realWinSize.height - 20);
	Vector2i activeWinSize(realWinSize.width - 40, realWinSize.height - 40);

	bool warpNeeded = false;

	if (x < activeWinTopLeft.x) {
		x += activeWinSize.x;
		warpNeeded = true;
	} else if (x >= activeWinBottomRight.x) {
		x -= activeWinSize.x;
		warpNeeded = true;
	}

	if (y < activeWinTopLeft.y) {
		y += activeWinSize.y;
		warpNeeded = true;
	} else if (y >= activeWinBottomRight.y) {
		y -= activeWinSize.y;
		warpNeeded = true;
	}

	if (warpNeeded) {
		glutWarpPointer(x, y);
		uiEngine.mouseWarped = true;
	}

	uiEngine.mouseLastClick.x = x;
	uiEngine.mouseLastClick.y = y;

	if ((movement.x == 0) && (movement.y == 0)) return;

	static const float scale_rotation = 0.005f;
	static const float scale_translation = 0.0025f;

	switch (uiEngine.mouseState) {
		case 1: {
			// left button: rotation
			Vector3f axis((float) -movement.y, (float) -movement.x, 0.0f);
			float angle = scale_rotation * sqrtf((float) (movement.x * movement.x + movement.y * movement.y));
			Matrix3f rot = createRotation(axis, angle);
			uiEngine.freeviewPose.SetRT(rot * uiEngine.freeviewPose.GetR(), rot * uiEngine.freeviewPose.GetT());
			uiEngine.freeviewPose.Coerce();
			uiEngine.needsRefresh = true;
			break;
		}
		case 2: {
			// right button: translation in x and y direction
			uiEngine.freeviewPose.SetT(uiEngine.freeviewPose.GetT() +
			                           scale_translation * Vector3f((float) movement.x, (float) movement.y, 0.0f));
			uiEngine.needsRefresh = true;
			break;
		}
		case 3: {
			// middle button: translation along z axis
			uiEngine.freeviewPose.SetT(
					uiEngine.freeviewPose.GetT() + scale_translation * Vector3f(0.0f, 0.0f, (float) movement.y));
			uiEngine.needsRefresh = true;
			break;
		}
		default:
			break;
	}
}

void UIEngine_BPO::GlutMouseWheelFunction(int button, int dir, int x, int y) {
	UIEngine_BPO& uiEngine = UIEngine_BPO::Instance();

	static const float scale_translation = 0.05f;

	uiEngine.freeviewPose.SetT(
			uiEngine.freeviewPose.GetT() + scale_translation * Vector3f(0.0f, 0.0f, (dir > 0) ? -1.0f : 1.0f));
	uiEngine.needsRefresh = true;
}