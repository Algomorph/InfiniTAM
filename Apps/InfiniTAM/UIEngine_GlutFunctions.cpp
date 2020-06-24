//  ================================================================
//  Created by Gregory Kramida on 5/15/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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
#include "UIEngine.h"
#include "../../ITMLib/Utils/Analytics/BenchmarkUtilities.h"
#include "../../ITMLib/Engines/Main/MultiEngine.h"
#include "../../ITMLib/Engines/Main/BasicVoxelEngine.h"
#include "../../ITMLib/Engines/Telemetry/TelemetryRecorderLegacy.h"


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

void UIEngine::GlutDisplayFunction() {
	UIEngine& uiEngine = UIEngine::Instance();

	// get updated images from processing thread
	uiEngine.main_engine->GetImage(uiEngine.outImage[0], uiEngine.outImageType[0], &uiEngine.freeview_pose,
	                               &uiEngine.freeviewIntrinsics);

	for (int w = 1; w < NUM_WIN; w++) {
		uiEngine.main_engine->GetImage(uiEngine.outImage[w], uiEngine.outImageType[w]);
	}

	// do the actual drawing
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(1.0f, 1.0f, 1.0f);
	glEnable(GL_TEXTURE_2D);

	UChar4Image** showImgs = uiEngine.outImage;
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
	int previous_frame_index = uiEngine.current_frame_index - 1;
	if (previous_frame_index >= 0) {
		glRasterPos2f(0.775f, -0.900f);
		sprintf(str, "Frame %5d", previous_frame_index);
		Safe_GlutBitmapString(GLUT_BITMAP_HELVETICA_18, (const char*) str);
	}

	//print frame rate
	glRasterPos2f(0.85f, -0.962f);
	sprintf(str, "%04.2lf", uiEngine.current_frame_processing_time);
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
	        uiEngine.index_of_frame_to_end_before,
	        uiEngine.depthVideoWriter != nullptr ? "off" : "on");
	Safe_GlutBitmapString(GLUT_BITMAP_HELVETICA_12, (const char*) str);

	glutSwapBuffers();
	uiEngine.needs_refresh = false;
}
void UIEngine::GlutIdleFunction() {
	UIEngine& ui_engine = UIEngine::Instance();
	if (ui_engine.shutdown_requested) {
		ui_engine.main_loop_action = UIEngine::EXIT;
	}
	switch (ui_engine.main_loop_action) {
		case PROCESS_FRAME:
			ui_engine.ProcessFrame();
			ui_engine.current_frame_index++; //done with current frame, increment the frame counter
			ui_engine.main_loop_action = PROCESS_PAUSED;
			ui_engine.needs_refresh = true;
			break;
		case PROCESS_VIDEO:
			ui_engine.ProcessFrame();
			ui_engine.current_frame_index++;
			ui_engine.needs_refresh = true;
			break;
		case PROCESS_N_FRAMES:
			ui_engine.ProcessFrame();
			ui_engine.current_frame_index++;
			ui_engine.needs_refresh = true;
			if (ui_engine.current_frame_index >=
			    ui_engine.index_of_frame_to_end_before) {
				ui_engine.main_loop_action = ui_engine.exit_after_automatic_run ? EXIT : PROCESS_PAUSED;
				if (ui_engine.save_after_automatic_run) {
					ui_engine.main_engine->SaveToFile(ui_engine.GeneratePreviousFrameOutputPath());
				}
				if (configuration::get().logging_settings.log_benchmarks) {
					benchmarking::log_all_timers();
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

	if (ui_engine.needs_refresh) {
		glutPostRedisplay();
	}
}

void UIEngine::GlutKeyUpFunction(unsigned char key, int x, int y) {
	UIEngine& ui_engine = UIEngine::Instance();
	int modifiers = glutGetModifiers();

	switch (key) {
		//TODO: rearrange in asciibeditc order (except fall-through cases) to make maintenance easier
		case 'i':
			//TODO: fix repeated interval functionality
			printf("processing %d frames ...\n", ui_engine.index_of_frame_to_end_before - ui_engine.auto_interval_frame_start);
			ui_engine.main_loop_action = UIEngine::PROCESS_N_FRAMES;
			break;
		case 'b':
			printf("processing input source ...\n");
			ui_engine.main_loop_action = UIEngine::PROCESS_VIDEO;
			break;
		case 'n':
			ui_engine.PrintProcessingFrameHeader();
			ui_engine.main_loop_action = UIEngine::PROCESS_FRAME;
			break;
		case 'k':
			if (ui_engine.image_recording_enabled) {
				printf("stopped recoding to disk ...\n");
				ui_engine.image_recording_enabled = false;
			} else {
				printf("started recoding to disk ...\n");
				ui_engine.processed_frame_count = 0;
				ui_engine.image_recording_enabled = true;
			}
			break;
		case 'v':
			if (modifiers & GLUT_ACTIVE_ALT) {
				if ((ui_engine.reconstructionVideoWriter != nullptr)) {
					printf("stopped recoding reconstruction video\n");
					delete ui_engine.reconstructionVideoWriter;
					ui_engine.reconstructionVideoWriter = nullptr;
				} else {
					printf("started recoding reconstruction video\n");
					ui_engine.reconstructionVideoWriter = new FFMPEGWriter();
				}
			} else {
				if ((ui_engine.rgbVideoWriter != nullptr) || (ui_engine.depthVideoWriter != nullptr)) {
					printf("stopped recoding input video\n");
					delete ui_engine.rgbVideoWriter;
					delete ui_engine.depthVideoWriter;
					ui_engine.rgbVideoWriter = nullptr;
					ui_engine.depthVideoWriter = nullptr;
				} else {
					printf("started recoding input video\n");
					ui_engine.rgbVideoWriter = new FFMPEGWriter();
					ui_engine.depthVideoWriter = new FFMPEGWriter();
				}
			}
			break;
		case 'q':
		case 'e':
		case 27: // esc key
			printf("exiting ...\n");
			ui_engine.main_loop_action = UIEngine::EXIT;
			break;
		case 'f':
			ui_engine.current_colour_mode = 0;
			//TODO: replace this whole if/else block with a separate function, use this function during initialization as well -Greg (Github: Algomorph)
			if (ui_engine.freeview_active) {
				ui_engine.outImageType[0] = MainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
				ui_engine.outImageType[1] = MainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;

				ui_engine.freeview_active = false;
			} else {
				ui_engine.outImageType[0] = MainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED;
				ui_engine.outImageType[1] = MainEngine::InfiniTAM_IMAGE_SCENERAYCAST;

				ui_engine.freeview_pose.SetFrom(ui_engine.main_engine->GetTrackingState()->pose_d);
				if (ui_engine.main_engine->GetView() != nullptr) {
					ui_engine.freeviewIntrinsics = ui_engine.main_engine->GetView()->calib.intrinsics_d;
					ui_engine.outImage[0]->ChangeDims(ui_engine.main_engine->GetView()->depth->dimensions);
				}

				switch (ui_engine.indexing_method) {
					case configuration::INDEX_HASH: {
						auto* multiEngine = dynamic_cast<MultiEngine<TSDFVoxel, VoxelBlockHash>*>(ui_engine.main_engine);
						if (multiEngine != nullptr) {
							int idx = multiEngine->findPrimaryLocalMapIdx();
							if (idx < 0) idx = 0;
							multiEngine->setFreeviewLocalMapIdx(idx);
						}
					}
						break;
					case configuration::INDEX_ARRAY:
						auto* multiEngine = dynamic_cast<MultiEngine<TSDFVoxel, PlainVoxelArray>*>(ui_engine.main_engine);
						if (multiEngine != nullptr) {
							int idx = multiEngine->findPrimaryLocalMapIdx();
							if (idx < 0) idx = 0;
							multiEngine->setFreeviewLocalMapIdx(idx);
						}
						break;
				}


				ui_engine.freeview_active = true;
			}
			ui_engine.needs_refresh = true;
			break;
		case 'c':
			ui_engine.current_colour_mode++;
			if (((ui_engine.freeview_active) &&
			     ((unsigned) ui_engine.current_colour_mode >= ui_engine.colourModes_freeview.size())) ||
			    ((!ui_engine.freeview_active) &&
			     ((unsigned) ui_engine.current_colour_mode >= ui_engine.colourModes_main.size())))
				ui_engine.current_colour_mode = 0;
			ui_engine.needs_refresh = true;
			break;
		case 't': {
			ui_engine.integration_active = !ui_engine.integration_active;
			ui_engine.main_engine->TurnOffIntegration();
		}
			break;
		case 'r': {
			ui_engine.main_engine->ResetAll();
		}
			break;
		case 's': {
			if (modifiers && GLUT_ACTIVE_ALT) {
				printf("saving scene to model ... ");
				ui_engine.main_engine->SaveVolumeToMesh("mesh.stl");
				printf("done\n");
			} else {
				printf("saving scene to disk ... ");
				try {
					ui_engine.main_engine->SaveToFile();
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
				ui_engine.main_engine->LoadFromFile(ui_engine.GenerateCurrentFrameOutputPath());
				printf("done\n");
			}
			catch (const std::runtime_error& e) {
				printf("failed: %s\n", e.what());
			}
		}
			break;
		case 'p':
			ui_engine.main_loop_action = PROCESS_PAUSED;
			break;
		case '[':
		case ']': {
			auto* multiEngineVBH = dynamic_cast<MultiEngine<TSDFVoxel, VoxelBlockHash>*>(ui_engine.main_engine);
			if (multiEngineVBH != nullptr) {
				int idx = multiEngineVBH->getFreeviewLocalMapIdx();
				if (key == '[') idx--;
				else idx++;
				multiEngineVBH->changeFreeviewLocalMapIdx(&(ui_engine.freeview_pose), idx);
				ui_engine.needs_refresh = true;
			}
			auto* multiEnginePVA = dynamic_cast<MultiEngine<TSDFVoxel, PlainVoxelArray>*>(ui_engine.main_engine);
			if (multiEnginePVA != nullptr) {
				int idx = multiEnginePVA->getFreeviewLocalMapIdx();
				if (key == '[') idx--;
				else idx++;
				multiEnginePVA->changeFreeviewLocalMapIdx(&(ui_engine.freeview_pose), idx);
				ui_engine.needs_refresh = true;
			}
		}
			break;
	}
	if (ui_engine.freeview_active) ui_engine.outImageType[0] = ui_engine.colourModes_freeview[ui_engine.current_colour_mode].type;
	else ui_engine.outImageType[0] = ui_engine.colourModes_main[ui_engine.current_colour_mode].type;
}

void UIEngine::GlutMouseButtonFunction(int button, int state, int x, int y) {
	UIEngine& uiEngine = UIEngine::Instance();

	if (state == GLUT_DOWN) {
		switch (button) {
			case GLUT_LEFT_BUTTON:
				uiEngine.current_mouse_operation = VIEW_ROTATION;
				break;
			case GLUT_MIDDLE_BUTTON:
				uiEngine.current_mouse_operation = VIEW_PANNING;
				break;
			case GLUT_RIGHT_BUTTON:
				uiEngine.current_mouse_operation = VIEW_DOLLYING;
				break;
			default:
				break;
		}
		uiEngine.last_mouse_click_position.x = x;
		uiEngine.last_mouse_click_position.y = y;

		glutSetCursor(GLUT_CURSOR_NONE);
	} else if (state == GLUT_UP && !uiEngine.mouse_warped) {
		uiEngine.current_mouse_operation = IDLE;
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

void UIEngine::GlutMouseMoveFunction(int x, int y) {
	UIEngine& ui_engine = UIEngine::Instance();

	if (ui_engine.mouse_warped) {
		ui_engine.mouse_warped = false;
		return;
	}

	if (!ui_engine.freeview_active || ui_engine.current_mouse_operation == IDLE) return;

	Vector2i movement;
	movement.x = x - ui_engine.last_mouse_click_position.x;
	movement.y = y - ui_engine.last_mouse_click_position.y;

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
		ui_engine.mouse_warped = true;
	}

	ui_engine.last_mouse_click_position.x = x;
	ui_engine.last_mouse_click_position.y = y;

	if ((movement.x == 0) && (movement.y == 0)) return;

	static const float scale_rotation = 0.005f;
	static const float scale_translation = 0.0025f;

	switch (ui_engine.current_mouse_operation) {
		case VIEW_ROTATION: {
			// left button: rotation
			Vector3f axis((float) -movement.y, (float) -movement.x, 0.0f);
			float angle = scale_rotation * sqrtf((float) (movement.x * movement.x + movement.y * movement.y));
			Matrix3f rot = createRotation(axis, angle);
			ui_engine.freeview_pose.SetRT(rot * ui_engine.freeview_pose.GetR(), rot * ui_engine.freeview_pose.GetT());
			ui_engine.freeview_pose.Coerce();
			ui_engine.needs_refresh = true;
			break;
		}
		case VIEW_PANNING: {
			// right button: translation in x and y direction
			ui_engine.freeview_pose.SetT(ui_engine.freeview_pose.GetT() +
			                           scale_translation * Vector3f((float) movement.x, (float) movement.y, 0.0f));
			ui_engine.needs_refresh = true;
			break;
		}
		case VIEW_DOLLYING: {
			// middle button: translation along z axis
			ui_engine.freeview_pose.SetT(
					ui_engine.freeview_pose.GetT() + scale_translation * Vector3f(0.0f, 0.0f, (float) movement.y));
			ui_engine.needs_refresh = true;
			break;
		}
		default:
			break;
	}
}

void UIEngine::GlutMouseWheelFunction(int button, int dir, int x, int y) {
	UIEngine& uiEngine = UIEngine::Instance();

	static const float scale_translation = 0.05f;

	uiEngine.freeview_pose.SetT(
			uiEngine.freeview_pose.GetT() + scale_translation * Vector3f(0.0f, 0.0f, (dir > 0) ? -1.0f : 1.0f));
	uiEngine.needs_refresh = true;
}