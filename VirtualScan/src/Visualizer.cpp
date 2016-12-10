
#include "stdafx.h"
#include <cmath>

#include "GlobalAppState.h"

#include "omp.h"


Visualizer::~Visualizer()
{
}



void Visualizer::init(ApplicationData& app)
{
	std::cout << "loading scene... ";
	m_scene.loadFromGlobaAppState(app.graphics, GlobalAppState::get());

	std::cout << "done!" << std::endl;

	std::cout << "scene bounds:\n" << m_scene.getBoundingBox() << std::endl;

	
	//vec3f eye = vec3f(
	//	m_scene.getBoundingBox().getMinX(), 
	//	0.5f*(m_scene.getBoundingBox().getMinY() + m_scene.getBoundingBox().getMaxY()), 
	//	0.5f*(m_scene.getBoundingBox().getMinZ() + m_scene.getBoundingBox().getMaxZ())
	//	);
	//eye = eye - 3.0f*(m_scene.getBoundingBox().getCenter() - eye);	//look from the outside
	//vec3f lookDir = m_scene.getBoundingBox().getCenter() - eye;
	
	vec3f eye = m_scene.getBoundingBox().getCenter();
	vec3f worldUp = vec3f::eY;
	vec3f lookDir = vec3f::eX;
	m_camera = Cameraf(eye, lookDir, worldUp, GlobalAppState::get().s_cameraFov, (float)app.window.getWidth() / app.window.getHeight(), 0.04f, 200.0f);	//in theory that should be depth min/max...

	m_font.init(app.graphics, "Calibri");


	m_renderTarget.load(app.graphics.castD3D11(), app.window.getWidth(), app.window.getHeight());
}




void Visualizer::render(ApplicationData& app)
{
	

	m_timer.frame();

	m_scene.render(m_camera);


	m_font.drawString("FPS: " + convert::toString(m_timer.framesPerSecond()), vec2i(10, 5), 24.0f, RGBColor::Red);

	//if (m_bEnableRecording) {
	//	m_recordedCameras.push_back(m_camera);
	//	m_font.drawString(app.graphics, "RECORDING ON " + std::to_string(m_recordedCameras.size()), vec2i(10, 30), 24.0f, RGBColor::Red);
	//}
}


void Visualizer::resize(ApplicationData &app)
{
	m_camera.updateAspectRatio((float)app.window.getWidth() / app.window.getHeight());
}

void Visualizer::keyDown(ApplicationData& app, UINT key)
{

	if (key == KEY_I) {
		m_scene.randomizeLighting();
	}

	//if (key == KEY_T) {
	//	if (m_recordedCameras.size() > 0) {

	//		SensorData sd;
	//		sd.m_sensorName = "VirtualScan";
	//		
	//		sd.m_colorWidth = m_renderTarget.getWidth();
	//		sd.m_colorHeight = m_renderTarget.getHeight();
	//		sd.m_depthWidth = m_renderTarget.getWidth();
	//		sd.m_depthHeight = m_renderTarget.getHeight();
	//		sd.m_depthShift = 1000.0f;	

	//		sd.m_calibrationDepth.setMatrices(
	//			Cameraf::graphicsToVisionProj(m_camera.getPerspective(), sd.m_depthWidth, sd.m_depthHeight)
	//			);
	//		sd.m_calibrationColor.setMatrices(
	//			Cameraf::graphicsToVisionProj(m_camera.getPerspective(), sd.m_colorWidth, sd.m_colorHeight)
	//			);

	//		sd.m_calibrationDepth.m_intrinsic(1, 1) *= -1.0f;
	//		sd.m_calibrationColor.m_intrinsic(1, 1) *= -1.0f;

	//		sd.m_colorCompressionType = SensorData::TYPE_JPEG;
	//		sd.m_depthCompressionType = SensorData::TYPE_ZLIB_USHORT;

	//		for (auto& camera : m_recordedCameras) {
	//			m_renderTarget.clear();
	//			m_renderTarget.bind();

	//			m_scene.render(camera);	//render call

	//			m_renderTarget.unbind();
	//			ml::ColorImageR8G8B8A8 color;
	//			ml::DepthImage32 depth;
	//			m_renderTarget.captureColorBuffer(color);
	//			m_renderTarget.captureDepthBuffer(depth, m_camera.getPerspective());

	//			ColorImageR8G8B8 c(color.getWidth(), color.getHeight());
	//			DepthImage16 d(depth.getWidth(), depth.getHeight());
	//			for (size_t i = 0; i < color.getWidth()*color.getHeight(); i++)
	//				c.getData()[i] = color.getData()[i].getVec3();
	//			for (size_t i = 0; i < depth.getWidth()*depth.getHeight(); i++)
	//				d.getData()[i] = math::round(depth.getData()[i] * sd.m_depthShift);
	//			sd.addFrame(c.getData(), d.getData(), camera.getCamera());
	//		}

	//		std::cout << sd << std::endl;
	//		std::cout << sd.m_calibrationColor.m_intrinsic << std::endl;
	//		std::cout << sd.m_calibrationDepth.m_intrinsic << std::endl;
	//		sd.saveToFile("test.sens");

	//		m_recordedCameras.clear();
	//		m_bEnableRecording = false;
	//	}
	//}

	if (key == KEY_F1) {		

		ml::D3D11RenderTarget renderTarget;
		renderTarget.load(app.graphics.castD3D11(), app.window.getWidth(), app.window.getHeight());
		renderTarget.clear();
		renderTarget.bind();
		
		m_scene.render(m_camera);	//render call

		renderTarget.unbind();
		ml::ColorImageR8G8B8A8 color;
		ml::DepthImage32 depth;
		renderTarget.captureColorBuffer(color);
		renderTarget.captureDepthBuffer(depth, m_camera.getPerspective());
		FreeImageWrapper::saveImage("color.png", color);
		FreeImageWrapper::saveImage("depth.png", ml::ColorImageR32G32B32A32(depth));

		{
			ml::PointCloudf pc;
			for (auto &p : depth) {
				if (p.value != depth.getInvalidValue()) {
					mat4f c = Cameraf::graphicsToVisionProj(m_camera.getPerspective(), depth.getWidth(), depth.getHeight()).getInverse();
					vec3f point = c * vec3f(p.x*p.value, p.y*p.value, p.value);
					pc.m_points.push_back(point);
				}
			}
			ml::PointCloudIOf::saveToFile("test.ply", pc);
		}
		

		{
			renderTarget.captureDepthBuffer(depth);
			mat4f projToCamera = m_camera.getPerspective().getInverse();
			mat4f cameraToWorld = m_camera.getCamera().getInverse();
			mat4f projToWorld = cameraToWorld * projToCamera;

			for (auto &p : depth) {
				if (p.value != 0.0f && p.value != 1.0f) {
					vec3f posProj = vec3f(app.graphics.castD3D11().pixelToNDC(vec2i((int)p.x, (int)p.y), depth.getWidth(), depth.getHeight()), p.value);
					vec3f posCamera = projToCamera * posProj;
					p.value = posCamera.z;
				}
				else {
					p.value = depth.getInvalidValue();
				}
			}
			FreeImageWrapper::saveImage("color1.png", color);
			FreeImageWrapper::saveImage("depth1.png", ml::ColorImageR32G32B32A32(depth));
		}

		std::cout << "screenshot taken (color.png / depth.png)" << std::endl;
	}
}

void Visualizer::keyPressed(ApplicationData &app, UINT key)
{
	const float distance = 0.1f;
	const float theta = 0.5f;

	if (key == KEY_S) m_camera.move(-distance);
	if (key == KEY_W) m_camera.move(distance);
	if (key == KEY_A) m_camera.strafe(-distance);
	if (key == KEY_D) m_camera.strafe(distance);
	if (key == KEY_E) m_camera.jump(distance);
	if (key == KEY_Q) m_camera.jump(-distance);

	if (key == KEY_UP) m_camera.lookUp(theta);
	if (key == KEY_DOWN) m_camera.lookUp(-theta);
	if (key == KEY_LEFT) m_camera.lookRight(theta);
	if (key == KEY_RIGHT) m_camera.lookRight(-theta);

	//if (key == KEY_Z) m_camera.roll(theta);
	//if (key == KEY_X) m_camera.roll(-theta);
}

void Visualizer::mouseDown(ApplicationData &app, MouseButtonType button)
{

}

void Visualizer::mouseWheel(ApplicationData &app, int wheelDelta)
{
	const float distance = 0.01f;
	m_camera.move(distance * wheelDelta);
}

void Visualizer::mouseMove(ApplicationData &app)
{
	const float distance = 0.05f;
	const float theta = 0.5f;

	vec2i posDelta = app.input.mouse.pos - app.input.prevMouse.pos;

	if (app.input.mouse.buttons[MouseButtonRight])
	{
		m_camera.strafe(-distance * posDelta.x);
		m_camera.jump(distance * posDelta.y);
	}

	if (app.input.mouse.buttons[MouseButtonLeft])
	{
		m_camera.lookRight(-theta * posDelta.x);
		m_camera.lookUp(theta * posDelta.y);
	}

}
