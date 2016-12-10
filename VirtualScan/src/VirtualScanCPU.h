#pragma once

#include "stdafx.h"

class VirtualScan
{
public:
	VirtualScan(void);
	~VirtualScan(void);

	//! currently depth & color cameras have the same params
	void initialize(const TriMeshf& mesh, const mat4f& perspective, UINT width, UINT height, const mat4f depthToColorExtrinsics = mat4f::identity());
	void initialize(const TriMeshf& mesh, const mat4f& perspective, UINT depthWidth, UINT depthHeight, UINT colorWidth, UINT colorHeight, const mat4f depthToColorExtrinsics = mat4f::identity());

	void virtualScan(const mat4f& camera, bool filterDepth, bool addNoiseToDepth, bool saveImages = false);

	void virtualScan360(void);
	void virtualScanTranslate(const vec3f& direction, float increment, unsigned int numSteps);
	void virtualScanStatic(const mat4f& initialTransformInverse, unsigned int numFrames = 50);
	void virtualScanTrajectory(const std::vector<mat4f>& trajectory);

	//! girl
	void virtualScanComplete(void);

	//void setUseScreenSpaceNormals(bool b) { m_useScreenSpaceNormals = b; }
	void setScanning(bool s, const mat4f& initialTransformInverse = mat4f::identity());
	bool isScanning(void) const { return m_scanning; }
	void setUseSphericalHarmonicsLighting(bool b) {
		m_useSphericalHarmonicsLighting = b;
	}

	void startScanning(const mat4f& initialTransformInverse = mat4f::identity());
	void stopScanning(void)  { m_scanning = false; }
	void stopScanningAndSaveToFile(const std::string& filename);

	void reset(const mat4f& transform = mat4f::identity());
	void saveRecordedFramesToFile(const std::string& filename);

	//! todo different
	unsigned int getDepthWidth(void) const  { return m_depthWidth; }
	unsigned int getDepthHeight(void) const { return m_depthHeight; }
	unsigned int getColorWidth(void) const  { return m_colorWidth; }
	unsigned int getColorHeight(void) const { return m_colorHeight; }

	const mat4f& getDepthIntrinsics(void) const { return m_depthIntrinsics; }
	const mat4f& getDepthExtrinsics(void) const { return m_depthToColorExtrinsics; } 
	const mat4f& getColorIntrinsics(void) const { return m_colorIntrinsics; }
	const mat4f& getColorExtrinsics(void) const { return mat4f::identity(); }

private:

	void virtualScanBoth(const mat4f &camToWorld, const mat4f& camToWorldColor, std::vector<const TriMeshAcceleratorBVHf*> accelVec, const mat4f& worldToCam, 
		float* depthFrame, vec4uc* colorFrame, unsigned int &invalidLow, unsigned int &invalidHigh);
	void virtualScanDepth(const mat4f &camToWorld, std::vector<const TriMeshAcceleratorBVHf*> accelVec, const mat4f& worldToCam, 
		float* depthFrame);
	void virtualScanColor(const mat4f &camToWorld, std::vector<const TriMeshAcceleratorBVHf*> accelVec, 
		vec4uc* colorFrame, unsigned int &invalidLow, unsigned int &invalidHigh);

	vec4uc getColor(TriMeshRayAcceleratorf::Intersection &intersect, const vec3f& eye, unsigned int &invalidLow, unsigned int &invalidHigh);


	void initializeParams(UINT width, UINT height, const mat4f& perspective, const mat4f& depthToColorExtrinsics = mat4f::identity());
	void initializeParams(UINT depthWidth, UINT depthHeight, UINT colorWidth, UINT colorHeight, const mat4f& perspective, const mat4f& depthToColorExtrinsics = mat4f::identity());
	void initializeMesh(const TriMeshf& mesh);

	void initializeLightingCoefficients(void);
	void evaluateH(const vec3f& normal, std::vector<float>& H) const;
	float evaluateLightingModel(const vec3f& normal) const;

	void addGaussianNoiseToDepthMap(float* depth, float sigma);
	void bilateralFilterDepthMap(float* depth, float* depthNew);
	float gaussR(float sigma, float dist) {
		return exp(-(dist*dist)/(2.0f*sigma*sigma));
	}
	float gaussD(float sigma, int x, int y) {
		return exp(-((x*x+y*y)/(2.0f*sigma*sigma)));
	}

	bool m_scanning;

	unsigned int m_depthWidth;
	unsigned int m_depthHeight;
	unsigned int m_colorWidth;
	unsigned int m_colorHeight;

	mat4f m_initialTransformInverse;
	mat4f m_depthIntrinsics;
	mat4f m_depthIntrinsicsInverse;
	mat4f m_colorIntrinsics;
	mat4f m_colorIntrinsicsInverse;
	mat4f m_depthToColorExtrinsics;
	mat4f m_depthToColorExtrinsicsInverse;
	//static mat4f s_d3dOffsetTransform;

	// scene
	TriMeshf m_mesh;
	TriMeshAcceleratorBVHf m_accel;

	std::vector<float> lightingCoeffs;

	std::list<vec4uc*> m_colorData;
	std::list<float*>  m_depthData;
	std::vector<mat4f> m_trajectory;

	bool m_useSphericalHarmonicsLighting;
};