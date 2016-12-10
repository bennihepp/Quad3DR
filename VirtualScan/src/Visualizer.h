#pragma once 


#include <sys/timeb.h>
#include <time.h>

class Visualizer : public ApplicationCallback
{
public:
	Visualizer() : ApplicationCallback() {}
	~Visualizer();	

	void init(ApplicationData &app);
	void render(ApplicationData &app);

	void keyDown(ApplicationData &app, UINT key);
	void keyPressed(ApplicationData &app, UINT key);
	void mouseDown(ApplicationData &app, MouseButtonType button);
	void mouseMove(ApplicationData &app);
	void mouseWheel(ApplicationData &app, int wheelDelta);
	void resize(ApplicationData &app);

private:
	FrameTimer m_timer;

	Scene m_scene;
	D3D11Font m_font;
	Cameraf m_camera;
	D3D11RenderTarget m_renderTarget;

};