// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// A very simple example that can be used as template project for
// a Chrono::Engine simulator with 3D view.
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_postprocess/ChPovRay.h"
#include "chrono_thirdparty/filesystem/path.h"

// #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::postprocess;
// using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {

    // Create output directory
    std::string out_dir = GetChronoOutputPath() + "POVRAY_1";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);
    ChSystemNSC sys;

    // Create an exporter to POVray
    ChPovRay pov_exporter = ChPovRay(&sys);

    // Important: set the path to the template:
    pov_exporter.SetTemplateFile(GetChronoDataFile("POVRay_chrono_template.pov"));

    // Set the path where it will save all .pov, .ini, .asset and .dat files, a directory will be created if not
    // existing
    pov_exporter.SetBasePath(out_dir);


    // Optional: change the default naming of the generated files:
    // pov_exporter.SetOutputScriptFile("rendering_frames.pov");
    // pov_exporter.SetOutputDataFilebase("my_state");
    // pov_exporter.SetPictureFilebase("picture");

    // --Optional: modify default light
    pov_exporter.SetLight(ChVector3d(-3, 4, 2), ChColor(0.15f, 0.15f, 0.12f), false);


    // --Optional: add further POV commands, for example in this case:
    //     create an area light for soft shadows
    //     create a Grid object; Grid() parameters: step, linewidth, linecolor, planecolor
    //   Remember to use \ at the end of each line for a multiple-line string.
    pov_exporter.SetCustomPOVcommandsScript(
        " \
	light_source {   \
      <2, 10, -3>  \
	  color rgb<1.2,1.2,1.2> \
      area_light <4, 0, 0>, <0, 0, 4>, 8, 8 \
      adaptive 1 \
      jitter\
    } \
	object{ Grid(1,0.02, rgb<0.7,0.8,0.8>, rgbt<1,1,1,1>) rotate <0, 0, 90>  } \
    ");

    /// [POV exporter]
    /* End example */
    
    // Create a Chrono physical system
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Pendulum example ------------------------------------

    // 1 - Create a floor that is fixed (that is used also to represent the absolute reference)

    auto floorBody = std::make_shared<ChBodyEasyBox>(10, 2, 10,  // x, y, z dimensions
                                                     3000,       // density
                                                     true,       // create visualization asset
                                                     false       // no collision geometry
                                                     );
    floorBody->SetPos(ChVector3d(0, -2, 0));
    floorBody->SetFixed(true);

    sys.Add(floorBody);

    // 2 - Create a pendulum

    auto pendulumBody = std::make_shared<ChBodyEasyBox>(0.5, 2, 0.5,  // x, y, z dimensions
                                                        3000,         // density
                                                        true,         // create visualization asset
                                                        false         // no collision geometry
                                                        );
    pendulumBody->SetPos(ChVector3d(0, 3, 0));
    pendulumBody->SetLinVel(ChVector3d(1, 0, 0));

    sys.Add(pendulumBody);

    // 3 - Create a spherical constraint.
    //   Here we'll use a ChLinkMateGeneric, but we could also use ChLinkLockSpherical

    auto sphericalLink =
        std::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false);  // x,y,z,Rx,Ry,Rz constrains
    ChFrame<> link_position_abs(ChVector3d(0, 4, 0));

    sphericalLink->Initialize(pendulumBody,        // the 1st body to connect
                              floorBody,           // the 2nd body to connect
                              false,               // the two following frames are in absolute, not relative, coords.
                              link_position_abs,   // the link reference attached to 1st body
                              link_position_abs);  // the link reference attached to 2nd body

    sys.Add(sphericalLink);

    // Optionally, set color and/or texture for visual assets
    pendulumBody->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.5f, 0.25f));
    floorBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/checker1.png"), 2, 2);

    // ==Asset== Attach a video camera.
    // Note that a camera can also be attached to a moving object.
    auto camera = chrono_types::make_shared<ChCamera>();
    camera->SetAngle(50);
    camera->SetPosition(ChVector3d(0, 3, -10));
    camera->SetAimPoint(ChVector3d(0, 1, 0));
    camera->SetUpVector(ChVector3d(0, -1, 0));
    // floorBody->AddCamera(camera);
    // pendulumBody->AddCamera(camera);
    // sys.AddCamera(ChVector3d(2, 2, -5), ChVector3d(0, 1, 0));
    pov_exporter.SetCamera(ChVector3d(0, 3, -10), ChVector3d(0, 1, 0), 50);

    pov_exporter.AddAll();

    // 4 - Create the Irrlicht visualization system
    // ChVisualSystemIrrlicht vis;
    // vis.SetWindowSize(800, 600);
    // vis.SetWindowTitle("A simple project template");
    // vis.Initialize();
    // vis.AddLogo();
    // vis.AddSkyBox();
    // vis.AddTypicalLights();
    // vis.AddCamera(ChVector3d(2, 2, -5), ChVector3d(0, 1, 0));
    // vis.AttachSystem(&sys);

    // 5 - Simulation loop
    ChRealtimeStepTimer realtime_timer;
    double step_size = 5e-3;

    // while (vis.Run()) {
    //     // Render scene
    //     vis.BeginScene();
    //     vis.Render();
    //     vis.EndScene();

    //     // Perform the integration step
    //     sys.DoStepDynamics(step_size);

    //     // Spin in place to maintain soft real-time
    //     realtime_timer.Spin(step_size);
    // }

    pov_exporter.ExportScript();

    while (sys.GetChTime() < 1.5) {
        sys.DoStepDynamics(0.01);

        std::cout << "time= " << sys.GetChTime() << std::endl;

        // 2) Create the incremental nnnn.dat and nnnn.pov files that will be load
        //    by the pov .ini script in POV-Ray (do this at each simulation timestep)
        pov_exporter.ExportData();
    }

    return 0;
}
