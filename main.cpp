#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <iostream>
#include "scene.h"


Eigen::MatrixXd V;
Eigen::MatrixXi F;
igl::opengl::glfw::Viewer mgpViewer;

float currTime = 0;

//initial values
float timeStep = 0.02;
float CRCoeff= 1.0;
string projectileFile = "sphere.mesh";
string dataPath = "../data";
float projectileDensity = 1.5;

Scene scene;

vector<Eigen::MatrixXd> platV;
Eigen::MatrixXi boxF(12, 3);
Eigen::MatrixXi boxT(12, 4);
vector<Eigen::RowVector3d> platCOM;
Eigen::RowVector4d boxOrientation;
vector<Eigen::Vector3d> platColor;

//void createPlatform()
//{
//  double platWidth=100.0;
//  platCOM<<0.0,-5.0,-0.0;
//  platV.resize(9,3);
//  platF.resize(12,3);
//  platT.resize(12,4);
//  platV<<-platWidth,0.0,-platWidth,
//  -platWidth,0.0,platWidth,
//  platWidth,0.0,platWidth,
//  platWidth,0.0, -platWidth,
//  -platWidth,-platWidth/10.0,-platWidth,
//  -platWidth,-platWidth/10.0,platWidth,
//  platWidth,-platWidth/10.0,platWidth,
//  platWidth,-platWidth/10.0, -platWidth,
//  0.0,-platWidth/20.0, 0.0;
//  platF<<0,1,2,
//  2,3,0,
//  6,5,4,
//  4,7,6,
//  1,0,5,
//  0,4,5,
//  2,1,6,
//  1,5,6,
//  3,2,7,
//  2,6,7,
//  0,3,4,
//  3,7,4;
//  
//  platOrientation<<1.0,0.0,0.0,0.0;
//  
//  platT<<platF, VectorXi::Constant(12,8);
//}

void createPoolTable(RowVector3d position, double width=100.0, double length=200.0, double thickness=5.0)
{
	// F and T for any box if vertices are given in the right order
	boxF <<
		0, 1, 2,
		2, 3, 0,
		6, 5, 4,
		4, 7, 6,
		1, 0, 5,
		0, 4, 5,
		2, 1, 6,
		1, 5, 6,
		3, 2, 7,
		2, 6, 7,
		0, 3, 4,
		3, 7, 4;
	boxT << boxF, VectorXi::Constant(12, 8);
	boxOrientation = Eigen::RowVector4d(1.0, 0.0, 0.0, 0.0);
	Eigen::RowVector3d green = Eigen::RowVector3d(0.23, 0.71, 0.23);
	Eigen::RowVector3d brown = Eigen::RowVector3d(0.58, 0.36, 0.12);

	// green platform
	platCOM.push_back(position);
	Eigen::MatrixXd platformV(9, 3);
	platformV <<
		-width,  thickness, -length,
		-width,  thickness,  length,
		 width,  thickness,  length,
		 width,  thickness, -length,
		-width, -thickness, -length,
		-width, -thickness,  length,
		 width, -thickness,  length,
		 width, -thickness, -length,
		0.0, 0.0, 0.0;
	platV.push_back(platformV);
	platColor.push_back(green);

	// right border
	platCOM.push_back(position + Eigen::RowVector3d(width + thickness, thickness, 0.0));
	Eigen::MatrixXd bar1V(9, 3);
	bar1V <<
		-thickness, thickness * 2, -length,
		-thickness, thickness * 2, length,
		thickness, thickness * 2, length,
		thickness, thickness * 2, -length,
		-thickness, -thickness * 2, -length,
		-thickness, -thickness * 2, length,
		thickness, -thickness * 2, length,
		thickness, -thickness * 2, -length,
		0.0, 0.0, 0.0;
	platV.push_back(bar1V);
	platColor.push_back(brown);

	// left border
	platCOM.push_back(position + Eigen::RowVector3d(-width - thickness, thickness, 0.0));
	platV.push_back(bar1V);
	platColor.push_back(brown);

	// top border
	platCOM.push_back(position + Eigen::RowVector3d(0.0, thickness, length + thickness));
	Eigen::MatrixXd bar2V(9, 3);
	bar2V <<
		-width, thickness * 2, -thickness,
		-width, thickness * 2, thickness,
		width, thickness * 2, thickness,
		width, thickness * 2, -thickness,
		-width, -thickness * 2, -thickness,
		-width, -thickness * 2, thickness,
		width, -thickness * 2, thickness,
		width, -thickness * 2, -thickness,
		0.0, 0.0, 0.0;
	platV.push_back(bar2V);
	platColor.push_back(brown);

	// lower border
	platCOM.push_back(position + Eigen::RowVector3d(0.0, thickness, -length - thickness));
	platV.push_back(bar2V);
	platColor.push_back(brown);
}

// Creates the catapult mesh
// The angle of the sticks is hardcoded at 45 degrees each
void createAndAddCatapult(RowVector3d pos, double height, double width, double thickness)
{
	// Some parameters
	double cylMeshLength = 19.0; // approximated by looking at .mesh file
	double relBaseThickness = 1.3;
	RowVector3d catapultBrown = RowVector3d(0.46, 0.29, 0.1);

	// Fitting the function parameters to the given height/width (diregarding the thickness)
	double length = width / (2 * cos(45) * cylMeshLength); // ensures the width parameter holds
	double cylLength = cylMeshLength * length;
	double relBaseLength = (height - (cos(45) * cylLength)) / cylLength; // ensures the height parameter holds

	// Reading the mesh file
	MatrixXi cylT, cylF;
	MatrixXd cylV, baseV, stickV;
	igl::readMESH(dataPath + std::string("/") + "cylinder.mesh", cylV, cylT, cylF);
	baseV = cylV; stickV = cylV;

	// Base
	baseV.col(0) *= thickness * relBaseThickness;
	baseV.col(1) *= thickness * relBaseThickness;
	baseV.col(2) *= length * relBaseLength;
	RowVector4d straightUp = RowVector4d(0, 0, 0.7071068, 0.7071068);
	RowVector3d baseHeight = RowVector3d(0.0, cylLength * relBaseLength, 0.0);
	scene.addMesh(baseV, cylF.rowwise().reverse(), cylT, 10000.0, true, pos + 0.5 * baseHeight, straightUp, catapultBrown);

	// Sticks
	stickV.col(0) *= thickness;
	stickV.col(1) *= thickness;
	stickV.col(2) *= length;
	double rotHeight = cos(45) * cylLength * 0.5;
	RowVector4d leftRot = RowVector4d(0.2705981, 0.2705981, 0.6532815, 0.6532815);
	RowVector4d rightRot = RowVector4d(-0.2705981, -0.2705981, 0.6532815, 0.6532815);
	scene.addMesh(stickV, cylF.rowwise().reverse(), cylT, 10000.0, true, pos + RowVector3d( rotHeight, cylLength * relBaseLength + rotHeight, 0.0), leftRot, catapultBrown);  // left
	scene.addMesh(stickV, cylF.rowwise().reverse(), cylT, 10000.0, true, pos + RowVector3d(-rotHeight, cylLength * relBaseLength + rotHeight, 0.0), rightRot, catapultBrown); // right
}

void updateMeshes(igl::opengl::glfw::Viewer &viewer)
{
  //RowVector3d platColor; platColor<<0.8,0.8,0.8;
  //RowVector3d meshColor; meshColor<<0.8,0.2,0.2;
  viewer.core.align_camera_center(scene.meshes[0].currV);
  for (int i=0;i<scene.meshes.size();i++){
    viewer.data_list[i].clear();
    viewer.data_list[i].set_mesh(scene.meshes[i].currV, scene.meshes[i].F);
    viewer.data_list[i].set_face_based(true);
    viewer.data_list[i].set_colors(scene.meshes[i].color);
    viewer.data_list[i].show_lines=false;
  }
  viewer.data_list[0].show_lines=false;
  //viewer.data_list[0].set_colors(platColor.replicate(scene.meshes[0].F.rows(),1));
  viewer.data_list[0].set_face_based(true);
  //viewer.core.align_camera_center(scene.meshes[0].currV);
}

bool key_pressed(igl::opengl::glfw::Viewer &viewer, unsigned int key, int modifier)
{
	float step = 1;

	if (key == 'q')
	{
		if (scene.catapult.aiming) scene.catapult.stretchPoint -= scene.catapult.orientation * step;
		return true;
	}

	if (key == 'e')
	{
		if (scene.catapult.aiming) scene.catapult.stretchPoint += scene.catapult.orientation * step;
		return true;
	}

	if (key == 'd')
	{
		if (scene.catapult.aiming) scene.catapult.stretchPoint += (scene.catapult.corners.row(1) - scene.catapult.corners.row(0)).normalized() * step;
		return true;
	}

	if (key == 'a')
	{
		if (scene.catapult.aiming) scene.catapult.stretchPoint -= (scene.catapult.corners.row(1) - scene.catapult.corners.row(0)).normalized() * step;
		return true;
	}

	if (key == 's')
	{
		if (scene.catapult.aiming) scene.catapult.stretchPoint += RowVector3d(0, -1, 0) * step;
		return true;
	}

	if (key == 'w')
	{
		if (scene.catapult.aiming) scene.catapult.stretchPoint += RowVector3d(0, 1, 0) * step;
		return true;
	}

	return false;
}

bool key_down(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier)
{
	if (key == ' ')
	{
		viewer.core.is_animating = !viewer.core.is_animating;
		if (viewer.core.is_animating)
			cout << "Simulation running" << endl;
		else
			cout << "Simulation paused" << endl;
		return true;
	}

	if (key == 'N')
	{
		if (!viewer.core.is_animating) {
			scene.updateScene(timeStep, CRCoeff);
			currTime += timeStep;
			updateMeshes(viewer);
			std::cout << "currTime: " << currTime << std::endl;
			return true;
		}
	}

  if (key == 'C')
  {
	  if (scene.catapult.aiming) scene.catapult.shoot();
	  else if (scene.catapult.projectile == NULL) {
		  //MatrixXi objT, objF;
		  //MatrixXd objV;
		  //igl::readMESH(dataPath + std::string("/") + projectileFile, objV, objT, objF);
		  //scene.addMesh(objV, objF.rowwise().reverse(), objT, projectileDensity, 0, Vector3d(0, 40, 0), RowVector4d(0, 1, 0, 0), RowVector3d(0.9, 0.9, 0.9));
		  //viewer.append_mesh();
		  scene.catapult.move(scene.meshes[scene.cueBallIndex].COM, &(scene.meshes[0]), &(scene.meshes[1]), &(scene.meshes[2]));
		  scene.catapult.fill(&scene.meshes[scene.cueBallIndex]);
	  }
  }
  return false;
}




bool pre_draw(igl::opengl::glfw::Viewer &viewer)
{
	using namespace Eigen;
	using namespace std;

	if (viewer.core.is_animating) {
		scene.updateScene(timeStep, CRCoeff);
		currTime += timeStep;
		//cout <<"currTime: "<<currTime<<endl;
		updateMeshes(viewer);
	}

	//cout << "scene.catapult.corners" << endl << scene.catapult.corners << endl << "scene.catapult.stretchPoint" << endl << scene.catapult.stretchPoint << endl;
	viewer.data().add_edges(scene.catapult.corners, scene.catapult.stretchPoint.replicate(4,1), Eigen::RowVector3d(0, 0, 255));

	return false;
}

class CustomMenu : public igl::opengl::glfw::imgui::ImGuiMenu
{
  
  virtual void draw_viewer_menu() override
  {
    // Draw parent menu
    ImGuiMenu::draw_viewer_menu();
    
    // Add new group
    if (ImGui::CollapsingHeader("Algorithm Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
      ImGui::InputFloat("CR Coeff",&CRCoeff,0,0,3);
      ImGui::InputFloat("Drag Coeff",&dragCoeff,0,0,3);
      ImGui::InputFloat("Friction Coeff",&frictionCoeff,0,0,3);
      ImGui::InputDouble("A.x",&scene.catapult.stretchPoint(0));
      ImGui::InputDouble("A.y",&scene.catapult.stretchPoint(1));
      ImGui::InputDouble("A.z",&scene.catapult.stretchPoint(2));
	  ImGui::InputText("Data Path", dataPath);
	  ImGui::InputText("Projectile Mesh", projectileFile);
	  ImGui::InputFloat("Projectile Density", &projectileDensity, 0, 0, 3);
	  ImGui::InputFloat("Rest Length 1", &scene.catapult.restLength1, 0, 0, 3);
	  ImGui::InputFloat("Rest Length 2", &scene.catapult.restLength2, 0, 0, 3);
	  ImGui::InputFloat("Stifness 1", &scene.catapult.K1, 0, 0, 3);
	  ImGui::InputFloat("Stifness 2", &scene.catapult.K2, 0, 0, 3);
      
      
      if (ImGui::InputFloat("Time Step", &timeStep)) {
        mgpViewer.core.animation_max_fps = (((int)1.0/timeStep));
      }
    }
  }
};



int main(int argc, char *argv[])
{
  using namespace Eigen;
  using namespace std;
  
  
  // Load scene
  if (argc<3){
    cout<<"Please provide path (argument 1 and name of scene file (argument 2)!"<<endl;
    return 0;
  }
  dataPath = argv[1];
  cout<<"scene file: "<<std::string(argv[2])<<endl;

  // Initializing pool table and catapult
  double tableThickness = 5.0, catapultThickness = 0.7, catapultHeight = 70.0, catapultWidth = 50.0;
  Vector3d catapultPos = RowVector3d(0.0, 0.0, 0.0);
  createAndAddCatapult(catapultPos, catapultHeight, catapultWidth, catapultThickness); // these need to be the first three meshes
  createPoolTable(Eigen::Vector3d(0.0, 0.0 - tableThickness, 0.0), 120.0, 250.0, tableThickness);
  for (int i = 0; i < platV.size(); i++)
	scene.addMesh(platV[i], boxF, boxT, 10000.0, true, platCOM[i], boxOrientation, platColor[i]);

  //load scene from file
  scene.cueBallIndex = scene.meshes.size();
  scene.loadScene(std::string(argv[1]),std::string(argv[2]), catapultPos, catapultHeight, catapultWidth, catapultThickness * 6.0);

  scene.updateScene(0.0, CRCoeff);

  scene.meshes[0].isPoolTable = true;
  
  // Viewer Settings
  for (int i=0;i<scene.meshes.size();i++){
    if (i!=0)
      mgpViewer.append_mesh();
    //mgpViewer.data_list[i].set_mesh(scene.meshes[i].currV, scene.meshes[i].F);
  }
  //mgpViewer.core.align_camera_center(scene.meshes[0].currV);
  
  mgpViewer.callback_pre_draw = &pre_draw;
  mgpViewer.callback_key_down = &key_down;
  mgpViewer.callback_key_pressed = &key_pressed;
  mgpViewer.core.is_animating = false;
  mgpViewer.core.animation_max_fps = 50.;
  updateMeshes(mgpViewer);
  CustomMenu menu;
  mgpViewer.plugins.push_back(&menu);
  
  cout<<"Press [space] to toggle continuous simulation" << endl;
  cout<<"Press 'S' to advance time step-by-step"<<endl;
  
  mgpViewer.launch();
 
}
