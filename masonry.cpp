//
// MASONRY
//
// Program that simulates masonry using the Chrono::Engine multibody library
// from www.chronoengine.info
//

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <functional> 
#include <cctype>

// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;


//using namespace std;


// Some global variables

int    GLOBAL_save_each = 10;
double GLOBAL_max_simulation_time = 3.0;
bool   GLOBAL_load_forces = false; 
bool   GLOBAL_swap_zy = true;



// Load brick pattern from disk
// Create a bunch of ChronoENGINE rigid bodies 

void load_brick_file(ChSystem& mphysicalSystem, const char* filename, ChSharedPtr<ChMaterialSurface> mmaterial) {

    std::fstream fin(filename);
	if (!fin.good())
		throw ChException("ERROR opening .dat file with bricks: " + std::string(filename) + "\n");

    int added_bricks = 0;

    std::string line;
    
    // Parse the file line-by-line
	while(std::getline(fin, line)) 
	{
		//trims white space from the beginning of the string
		line.erase(line.begin(), find_if(line.begin(), line.end(), std::not1(std::ptr_fun<int, int>(std::isspace)))); 
        
        // skip empty lines
		if(line[0] == 0) 
            continue; 

        // skip comments
        if(line[0] == '#') {
			continue; 
		}

		// a normal line should contain brick data:
		if (true)
		{
			double tokenvals[300];
			int ntokens = 0;

			std::string token;
			std::istringstream ss(line);

            // parse line in format:
            // ID, Fx Fy, Fz, Ref.x, Ref.y, Ref.z, x,y,z, x,y,z, x,y,z, ..,..,..
			while(std::getline(ss, token,',') && ntokens < 300) 
			{
				std::istringstream stoken(token);
                //GetLog() << "  token n." << ntokens << " is: "<< stoken.str().c_str() << "\n";
				stoken >> tokenvals[ntokens]; 
				++ntokens;
			}
			++added_bricks;

            if ((ntokens-2) % 3 != 0)
                throw ChException("ERROR in .dat file, format is: ID, fixed, Fx, Fy, Fz, and three x y z coords, each per brick corner, see line:\n"+ line+"\n");

            int  my_ID    = (int)tokenvals[0];
            bool my_fixed = (bool)tokenvals[1];   
            int token_stride = 2;

            ChVector<> my_force;
            if (GLOBAL_load_forces) {
                my_force.x = tokenvals[token_stride+0];
                my_force.y = tokenvals[token_stride+1];
                my_force.z = tokenvals[token_stride+2];
                token_stride += 3;
                if (GLOBAL_swap_zy) std::swap(my_force.y, my_force.z);
            }

            ChVector<> my_reference;
            my_reference.x = tokenvals[token_stride+0];
            my_reference.y = tokenvals[token_stride+1];
            my_reference.z = tokenvals[token_stride+2];
            if (GLOBAL_swap_zy) std::swap(my_reference.y, my_reference.z);
            token_stride += 3;
            
            std::vector< ChVector<> > my_vertexes;
            for (int off = token_stride; off < ntokens; off += 3) {
                ChVector<> my_point;
                my_point.x = tokenvals[off+0];
                my_point.y = tokenvals[off+1];
                my_point.z = tokenvals[off+2];
                if (GLOBAL_swap_zy) std::swap(my_point.y, my_point.z);
                my_point = my_point - my_reference; // chrono want these points in reference system, but in file are in absolute system
                my_vertexes.push_back(my_point);
            }

            // Create a polygonal body:
            ChSharedPtr<ChBodyEasyConvexHullAuxRef> my_body (new ChBodyEasyConvexHullAuxRef(my_vertexes,1500,true,true));
            my_body->SetIdentifier(my_ID);
            my_body->Set_Scr_force(my_force);
            my_body->SetMaterialSurface(mmaterial);
            my_body->SetBodyFixed(my_fixed);
            my_body->SetFrame_REF_to_abs(ChFrame<>(my_reference));

            // Set the color of body by randomizing a gray shade
            ChSharedPtr<ChColorAsset> mcolor (new ChColorAsset);
            double mgray = 0.6+0.4*ChRandom();
            mcolor->SetColor(ChColor(mgray, mgray, mgray));
            my_body->AddAsset(mcolor);

            mphysicalSystem.Add(my_body);
		}


	} // end while

}


// The following is not used - just an example bout how to create 
// box briks programmatically.

void create_tile_pattern(ChSystem& mphysicalSystem) {
    // Create a material that will be shared between bricks
    ChSharedPtr<ChMaterialSurface> mmaterial_brick(new ChMaterialSurface);
    mmaterial_brick->SetFriction(0.4f);

    double bricksize_x = 2;
    double bricksize_z = 1;
    double bricksize_y = 2;
    int nrows_x= 6;
    int nrows_z= 18;
    ChVector<> planedispl(-0.5*nrows_x*(bricksize_x+2*bricksize_z), bricksize_y*0.5, -nrows_z*bricksize_z*0.5);

    // Create bricks in L pattern
    double noff_x= 0;
    for (int iz = 0; iz < nrows_z; iz += 1) {
        
        if (noff_x>= (bricksize_x+2*bricksize_z))
            noff_x = 0;

        for (int ix = 0; ix < nrows_x; ix += 1) {

            ChSharedPtr<ChBodyEasyBox> mrigidBody1(new ChBodyEasyBox(bricksize_x, bricksize_y, bricksize_z,
                                                                     1000,      // density
                                                                     true,      // collide enable?
                                                                     true));    // visualization?
            mrigidBody1->SetPos(planedispl+ChVector<>(noff_x + ix*(bricksize_x+(2*bricksize_z)) +0.5*bricksize_x, 0, iz*(bricksize_z)+0.5*bricksize_z));
            mrigidBody1->SetMaterialSurface(mmaterial_brick);  // use shared surface properties
            mphysicalSystem.Add(mrigidBody1);

            ChSharedPtr<ChTexture> mtexture(new ChTexture());
                mtexture->SetTextureFilename(GetChronoDataFile("cubetexture_borders.png"));
                mrigidBody1->AddAsset(mtexture);

            ChSharedPtr<ChBodyEasyBox> mrigidBody2(new ChBodyEasyBox(bricksize_z, bricksize_y, bricksize_x,
                                                                     1000,      // density
                                                                     true,      // collide enable?
                                                                     true));    // visualization?
            mrigidBody2->SetPos(planedispl+ChVector<>(noff_x + ix*(bricksize_x+(2*bricksize_z)) +0.5*bricksize_z, 0, iz*(bricksize_z)+0.5*bricksize_x+bricksize_z));
            mrigidBody2->SetMaterialSurface(mmaterial_brick);  // use shared surface properties
            mphysicalSystem.Add(mrigidBody2);

            mrigidBody2->AddAsset(mtexture);
        }
        noff_x +=bricksize_z;
    }

    // Create the floor using
    // fixed rigid body of 'box' type:
    // HAHAHA
   
    // Create a material for brick-floor
    ChSharedPtr<ChMaterialSurface> mmaterial_floor(new ChMaterialSurface);
    mmaterial_floor->SetFriction(0.0f);

    ChSharedPtr<ChBodyEasyBox> mrigidFloor(new ChBodyEasyBox(40, 4, 40,  // x,y,z size
                                                             1000,         // density
                                                             true,         // collide enable?
                                                             true));       // visualization?
    mrigidFloor->SetPos(ChVector<>(0, -2, 0));
    mrigidFloor->SetMaterialSurface(mmaterial_floor);
    mrigidFloor->SetBodyFixed(true);

    ChSharedPtr<ChTexture> mtexture(new ChTexture());
    mtexture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    mrigidFloor->AddAsset(mtexture);

    mphysicalSystem.Add(mrigidFloor);

}




// This is the contact reporter class, just for writing contacts on 
// a file on disk
class _contact_reporter_class : public  chrono::ChReportContactCallback2 
{
    public:
    ChStreamOutAsciiFile* mfile; // the file to save data into

    virtual bool ReportContactCallback2(
                                const ChVector<>& pA,             ///< get contact pA
                                const ChVector<>& pB,             ///< get contact pB
                                const ChMatrix33<>& plane_coord,  ///< get contact plane coordsystem (A column 'X' is contact normal)
                                const double& distance,           ///< get contact distance
                                const ChVector<>& react_forces,   ///< get react.forces (if already computed). In coordsystem 'plane_coord'
                                const ChVector<>& react_torques,  ///< get react.torques, if rolling friction (if already computed).
                                ChContactable* contactobjA,  ///< get model A (note: some containers may not support it and could be zero!)
                                ChContactable* contactobjB   ///< get model B (note: some containers may not support it and could be zero!)
        )
    {
        // For each contact, this function is executed. 
        // In this example, saves on ascii file:
        //   position xyz, direction xyz, normal impulse, tangent impulse U, tangent impulse V, modelA ID, modelB ID information is saved. 
        (*mfile)    << pA.x << ", " 
                    << pA.y << ", " 
                    << pA.z << ", " 
                    << plane_coord.Get_A_Xaxis().x << ", "
                    << plane_coord.Get_A_Xaxis().y << ", "
                    << plane_coord.Get_A_Xaxis().z << ", "
                    << react_forces.x << ", "
                    << react_forces.y << ", "
                    << react_forces.z << ", "
                    << contactobjA->GetPhysicsItem()->GetIdentifier() << ", "
                    << contactobjB->GetPhysicsItem()->GetIdentifier() << "\n";

        return true;  // to continue scanning contacts
    }
};



/// THE PROGRAM STARTS HERE!!!

int main(int argc, char* argv[]) {

    char* filename = "bricks.dat"; // commento per Vale: variabile stringa, inizializzata a default

    if (argc ==2)
        filename = argv[1];

    // Create a ChronoENGINE physical system
    ChSystem mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Bricks test", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice(), core::vector3df(70.f, 120.f, -90.f),
                                    core::vector3df(30.f, 80.f, 60.f), 290, 190);
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(-8, 8, 8), core::vector3df(0, 0, 0));

    // Here set the inward-outward margins for collision shapes:
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.005);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.000);

    //
    // HERE YOU POPULATE THE MECHANICAL SYSTEM OF CHRONO...
    //

    // The default material for the bricks:
    ChSharedPtr<ChMaterialSurface> mmaterial(new ChMaterialSurface);
    mmaterial->SetFriction(0.4f);
    //mmaterial->SetRestitution(0.0f);
    //mmaterial->SetCompliance(0.0000005f);
    //mmaterial->SetComplianceT(0.0000005f);
    //mmaterial->SetDampingF(0.2f);


    // Create all the rigid bodies loading their shapes from disk
    try {
        load_brick_file (mphysicalSystem, "bricks.dat", mmaterial);
    }
    catch (ChException my_load_error) {
        GetLog()<< my_load_error.what();
        system("pause");
    }


    // Use this function for adding a ChIrrNodeAsset to all items
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' into Irrlicht meshes the assets
    // into Irrlicht-visualizable meshes
    application.AssetUpdateAll();


    // Set no gravity on Y:
    application.GetSystem()->Set_G_acc(ChVector<>(0,-9.8,0));
    //application.GetSystem()->Set_G_acc(ChVector<>(0,0,0));

    // Prepare the physical system for the simulation

    mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);

    mphysicalSystem.SetMaxPenetrationRecoverySpeed(0.1); 
    mphysicalSystem.SetIterLCPmaxItersSpeed(60);
    mphysicalSystem.SetIterLCPwarmStarting(true);

    //
    // THE SOFT-REAL-TIME CYCLE
    //


    application.SetTimestep(0.01);
    application.SetPaused(true);

    while (application.GetDevice()->run()) {
        application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

        

        application.DrawAll();

        ChIrrTools::drawGrid(application.GetVideoDriver(), 2, 2, 10, 10,
                             ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI / 2, VECT_X)),
                             video::SColor(50, 90, 90, 150), true);

        application.DoStep();


        // Do some output to disk, for later postprocessing
        if (mphysicalSystem.GetStepcount() % GLOBAL_save_each  == 0)
        {
            // a) Use the contact callback object to save contacts:
            char contactfilename[200];
            sprintf(contactfilename, "%s%05d%s", "contacts", mphysicalSystem.GetStepcount(), ".txt");  // ex: contacts00020.tx
            _contact_reporter_class my_contact_rep;
            ChStreamOutAsciiFile result_contacts(contactfilename);
            my_contact_rep.mfile = &result_contacts;
            mphysicalSystem.GetContactContainer()->ReportAllContacts2(&my_contact_rep);

            // b) Save rigid body positions and rotations
            char bodyfilename[200];
            sprintf(bodyfilename, "%s%05d%s", "bodies", mphysicalSystem.GetStepcount(), ".txt");  // ex: bodies00020.tx
            ChStreamOutAsciiFile result_bodies(bodyfilename);
            ChSystem::IteratorBodies mbodies = mphysicalSystem.IterBeginBodies();
            while (mbodies != mphysicalSystem.IterEndBodies()) {
                result_bodies   << (*mbodies)->GetIdentifier()  << ", " 
                                << (*mbodies)->GetPos().x  << ", "
                                << (*mbodies)->GetPos().y  << ", "
                                << (*mbodies)->GetPos().z  << ", "
                                << (*mbodies)->GetRot().e0  << ", "
                                << (*mbodies)->GetRot().e1  << ", "
                                << (*mbodies)->GetRot().e2  << ", "
                                << (*mbodies)->GetRot().e3  << ", "
                                << (*mbodies)->GetRotAngle() * CH_C_RAD_TO_DEG  << "\n";
                ++mbodies;
            }
        }

        // Force the simulator to close after N seconds
        if (mphysicalSystem.GetChTime() > GLOBAL_max_simulation_time)
            application.GetDevice()->closeDevice();

        application.GetVideoDriver()->endScene();
    }

    return 0;
}
