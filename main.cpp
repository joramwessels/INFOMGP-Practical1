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

bool key_down(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier)
{
  if (key == ' ')
  {
    viewer.core.is_animating = !viewer.core.is_animating;
    if (viewer.core.is_animating)
      cout<<"Simulation running"<<endl;
    else
      cout<<"Simulation paused"<<endl;
    return true;
  }
  
  if (key == 'S')
  {
    if (!viewer.core.is_animating){
      scene.updateScene(timeStep, CRCoeff);
      currTime+=timeStep;
      updateMeshes(viewer);
      std::cout <<"currTime: "<<currTime<<std::endl;
      return true;
    }
  }
  return false;
}




bool pre_draw(igl::opengl::glfw::Viewer &viewer)
{
  using namespace Eigen;
  using namespace std;
  
  if (viewer.core.is_animating){
    scene.updateScene(timeStep, CRCoeff);
    currTime+=timeStep;
    //cout <<"currTime: "<<currTime<<endl;
    updateMeshes(viewer);
  }
 
  
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
  cout<<"scene file: "<<std::string(argv[2])<<endl;
  //create platform
  //createPlatform();
  createPoolTable(Eigen::Vector3d(0.0, 0.0, 0.0));
  for (int i = 0; i < platV.size(); i++)
	scene.addMesh(platV[i], boxF, boxT, 10000.0, true, platCOM[i], boxOrientation, platColor[i]);
  
  //load scene from file
  scene.loadScene(std::string(argv[1]),std::string(argv[2]));

  scene.updateScene(0.0, CRCoeff);
  
  // Viewer Settings
  for (int i=0;i<scene.meshes.size();i++){
    if (i!=0)
      mgpViewer.append_mesh();
    //mgpViewer.data_list[i].set_mesh(scene.meshes[i].currV, scene.meshes[i].F);
  }
  //mgpViewer.core.align_camera_center(scene.meshes[0].currV);
  
  
  mgpViewer.callback_pre_draw = &pre_draw;
  mgpViewer.callback_key_down = &key_down;
  mgpViewer.core.is_animating = false;
  mgpViewer.core.animation_max_fps = 50.;
  updateMeshes(mgpViewer);
  CustomMenu menu;
  mgpViewer.plugins.push_back(&menu);
  
  cout<<"Press [space] to toggle continuous simulation" << endl;
  cout<<"Press 'S' to advance time step-by-step"<<endl;
  
  mgpViewer.launch();
 
}
