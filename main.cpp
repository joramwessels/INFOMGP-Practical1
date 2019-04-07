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
string projectileFile = "box_tri.mesh";
string dataPath = "../data";
float projectileDensity = 2.5;

Scene scene;

Eigen::MatrixXd platV;
Eigen::MatrixXi platF;
Eigen::MatrixXi platT;
Eigen::RowVector3d platCOM;
Eigen::RowVector4d platOrientation;

void createPlatform()
{
  double platWidth=100.0;
  platCOM<<0.0,-5.0,-0.0;
  platV.resize(9,3);
  platF.resize(12,3);
  platT.resize(12,4);
  platV<<-platWidth,0.0,-platWidth,
  -platWidth,0.0,platWidth,
  platWidth,0.0,platWidth,
  platWidth,0.0, -platWidth,
  -platWidth,-platWidth/10.0,-platWidth,
  -platWidth,-platWidth/10.0,platWidth,
  platWidth,-platWidth/10.0,platWidth,
  platWidth,-platWidth/10.0, -platWidth,
  0.0,-platWidth/20.0, 0.0;
  platF<<0,1,2,
  2,3,0,
  6,5,4,
  4,7,6,
  1,0,5,
  0,4,5,
  2,1,6,
  1,5,6,
  3,2,7,
  2,6,7,
  0,3,4,
  3,7,4;
  
  platOrientation<<1.0,0.0,0.0,0.0;
  
  platT<<platF, VectorXi::Constant(12,8);
  
  
}

void updateMeshes(igl::opengl::glfw::Viewer &viewer)
{
  RowVector3d platColor; platColor<<0.8,0.8,0.8;
  RowVector3d meshColor; meshColor<<0.8,0.2,0.2;
  viewer.core.align_camera_center(scene.meshes[0].currV);
  for (int i=0;i<scene.meshes.size();i++){
    viewer.data_list[i].clear();
    viewer.data_list[i].set_mesh(scene.meshes[i].currV, scene.meshes[i].F);
    viewer.data_list[i].set_face_based(true);
    viewer.data_list[i].set_colors(meshColor);
    viewer.data_list[i].show_lines=false;
  }
  viewer.data_list[0].show_lines=false;
  viewer.data_list[0].set_colors(platColor.replicate(scene.meshes[0].F.rows(),1));
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
		  MatrixXi objT, objF;
		  MatrixXd objV;
		  igl::readMESH(dataPath + std::string("/") + projectileFile, objV, objT, objF);
		  scene.addMesh(objV, objF.rowwise().reverse(), objT, projectileDensity, 0, Vector3d(0, 40, 0), RowVector4d(0, 1, 0, 0));
		  viewer.append_mesh();
		  scene.catapult.fill(&scene.meshes.back());
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

bool post_draw(igl::opengl::glfw::Viewer &viewer)
{
	using namespace Eigen;
	using namespace std;

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
  cout<<"scene file: "<<std::string(argv[2])<<endl;
  //create platform
  createPlatform();
  scene.addMesh(platV, platF, platT, 10000.0, true, platCOM, platOrientation);
  
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
  mgpViewer.callback_post_draw = &post_draw;
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
