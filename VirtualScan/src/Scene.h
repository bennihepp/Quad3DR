#pragma once

#include "GlobalAppState.h"
#include "Lighting.h"

class RenderObject {
public:
	RenderObject(GraphicsDevice& g, const TriMeshf& triMesh, const Materialf& material, const mat4f& modelToWorld) {
		m_triMesh.load(g, triMesh);

		m_material = material;
		if (m_material.m_TextureFilename_Kd != "") {
			std::string texFile = m_material.m_TextureFilename_Kd;
			//that's a hack because I don't know how to load exrs (manually converted with imagemagick)
			if (util::getFileExtension(texFile) == "exr") {
				texFile = util::replace(texFile, ".exr", ".png");
			}

			if (util::fileExists(texFile)) {
				try {
				std::cout << "loading " << texFile << "... ";
					ColorImageR8G8B8A8 tex;
					FreeImageWrapper::loadImage(texFile, tex);
					//tex.setPixels(vec4uc(255, 0, 0, 255));	//debug
					m_texture.init(g, tex);
					std::cout << "done!" << std::endl;
				}
				catch (const std::exception& e) {
					std::cout << texFile << " : " << e.what() << std::endl;
				}
			}
			else {
				std::cout << "can't file tex file " << texFile << std::endl;
			}
		}

		m_modelToWorld = modelToWorld;
		for (const auto& v : triMesh.getVertices())  {
			m_boundingBoxWorld.include(m_modelToWorld * v.position);
		}
	}

	~RenderObject() {

	}

	const mat4f& getModelToWorld() const {
		return m_modelToWorld;
	}

	const D3D11TriMesh& getD3D11TriMesh() const {
		return m_triMesh;
	}

	const D3D11Texture2D &getD3D11Texture2D() const {
		return m_texture;
	}

	const BoundingBox3f& getBoundingBoxWorld() const {
		return m_boundingBoxWorld;
	}

	const Materialf& getMaterial() const {
		return m_material;
	}


	const bool isTextured() const {
		return m_texture.isLoaded();
	}

private:
	mat4f				m_modelToWorld;
	D3D11TriMesh		m_triMesh;
	D3D11Texture2D		m_texture;
	BoundingBox3f		m_boundingBoxWorld;
	
	Materialf			m_material;
};



class Scene
{
public:
	Scene() {

	}

	~Scene() {

	}

	void loadFromGlobaAppState(GraphicsDevice& g, const GlobalAppState& gas) {
		
		const std::vector<std::string> meshFilenames = gas.s_meshFilenames;
		m_graphics = &g;

		m_cbCamera.init(g);
		m_cbMaterial.init(g);
		m_lighting.loadFromGlobaAppState(g, gas);

		//TODO factor our the shader loading; ideally every object has a shader
		m_shaders.init(g);
		m_shaders.registerShader("shaders/phong.hlsl", "phong", "vertexShaderMain", "vs_4_0", "pixelShaderMain", "ps_4_0");
		m_shaders.registerShader("shaders/phong.hlsl", "phong_textured", "vertexShaderMain", "vs_4_0", "pixelShaderMain_textured", "ps_4_0");

		for (const std::string& meshFilename : meshFilenames) {
			MeshDataf meshDataAll = MeshIOf::loadFromFile(meshFilename);
			
			std::vector< std::pair <MeshDataf, Materialf > > meshDataByMaterial = meshDataAll.splitByMaterial();
			
			for (auto& m : meshDataByMaterial) {

				MeshDataf& meshData = m.first;
				Materialf& material = m.second;

				MLIB_ASSERT(meshData.isConsistent());
				if (!meshData.isTriMesh()) {
					std::cout << "Warning mesh " << meshFilename << " contains non-tri faces (auto-converting)" << std::endl;
					meshData.makeTriMesh();
				}


				MLIB_ASSERT(meshData.isConsistent());
				if (meshData.m_Colors.size() == 0) meshData.m_Colors.resize(meshData.m_Vertices.size(), vec4f(1.0f, 1.0f, 1.0f, 1.0f));	//set default color if none present
				TriMeshf triMesh(meshData);
				if (!triMesh.hasNormals())	triMesh.computeNormals();

				material.m_ambient = vec4f(0.1f);

				std::string path = util::directoryFromPath(meshFilename);
				if (material.m_TextureFilename_Kd != "") material.m_TextureFilename_Kd = path + material.m_TextureFilename_Kd;
				addObject(triMesh, material);
			} 

		}	

		
	}

	void addObject(const TriMeshf& triMesh, const Materialf& material, const mat4f& modelToWorld = mat4f::identity()) {
		m_objects.emplace_back(RenderObject(*m_graphics, triMesh, material, modelToWorld));
		m_boundingBox.include(m_objects.back().getBoundingBoxWorld());
	}


	void render(const Cameraf& camera) {

		m_lighting.updateAndBind(2);

		for (const RenderObject& o : m_objects) {
			ConstantBufferCamera cbCamera;
			cbCamera.worldViewProj = camera.getPerspective() * camera.getCamera() * o.getModelToWorld();
			cbCamera.world = o.getModelToWorld();
			cbCamera.eye = vec4f(camera.getEye());
			m_cbCamera.updateAndBind(cbCamera, 0);

			const Materialf material = o.getMaterial();

			ConstantBufferMaterial cbMaterial;
			cbMaterial.ambient = material.m_ambient;
			cbMaterial.diffuse = material.m_diffuse;
			cbMaterial.specular = material.m_specular;
			cbMaterial.shiny = material.m_shiny;
			m_cbMaterial.updateAndBind(cbMaterial, 1);

			if (o.isTextured()) {
				o.getD3D11Texture2D().bind(0);
				m_shaders.bindShaders("phong_textured");
			}
			else {
				m_shaders.bindShaders("phong");
			}
			o.getD3D11TriMesh().render();

			if (o.isTextured()) {
				o.getD3D11Texture2D().unbind(0);
			}
		}

	} 

	const BoundingBox3f& getBoundingBox() const {
		return m_boundingBox;
	}

	void randomizeLighting() {
		m_lighting.randomize();
	}

	const Lighting& getLighting() const {
		return m_lighting;
	}

	void setLighting(const Lighting& l) {
		m_lighting = l;
	}

private:

	struct ConstantBufferCamera {
		mat4f worldViewProj;
		mat4f world;
		vec4f eye;
	};

	struct ConstantBufferMaterial {
		vec4f ambient;
		vec4f diffuse;
		vec4f specular;
		float shiny;
		vec3f dummy;
	};


	GraphicsDevice* m_graphics;

	D3D11ShaderManager m_shaders;

	std::vector<RenderObject> m_objects;
	BoundingBox3f m_boundingBox;


	D3D11ConstantBuffer<ConstantBufferCamera>	m_cbCamera;
	D3D11ConstantBuffer<ConstantBufferMaterial> m_cbMaterial;
	Lighting m_lighting;
};

