#include "include/ManipulatorRobot.hpp"

using std::cout;
using std::endl;

namespace shared {

template<class T>
struct VecToList
{
    static PyObject* convert(const std::vector<T>& vec)
    {
        boost::python::list* l = new boost::python::list();
        for(size_t i = 0; i < vec.size(); i++)
            (*l).append(vec[i]);

        return l->ptr();
    }
};

bool ManipulatorRobot::initJoints(TiXmlElement *robot_xml) {
	for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
	{
		// Joint Names
		std::string joint_name(joint_xml->Attribute("name"));
		joint_names_.push_back(joint_name);
		
		// Joint origin
		std::vector<double> origin = process_origin_(joint_xml);
		joint_origins_.push_back(origin);
		
		// Joint axes
		std::vector<int> joint_axis;
		TiXmlElement *axis_xml = joint_xml->FirstChildElement("axis");
		if (axis_xml) {
			const char* xyz_str = axis_xml->Attribute("xyz");
			std::vector<std::string> pieces;					
			boost::split( pieces, xyz_str, boost::is_any_of(" "));
			for (unsigned int i = 0; i < pieces.size(); ++i) { 
			    if (pieces[i] != "") { 
			    	try {
			    		joint_axis.push_back(boost::lexical_cast<int>(pieces[i].c_str()));
			    	}
			    	catch (boost::bad_lexical_cast &e) {
			    								    									
			        }
			    }
			}
			
			joint_axes_.push_back(joint_axis);
		}
		else {
			std::vector<int> ax({0, 0, 0});
			joint_axes_.push_back(ax);
		}
		
		// Joint limits
		TiXmlElement *limit_xml = joint_xml->FirstChildElement("limit");
		double torque_limit = 0.0;
		double lower_limit = 0.0;
		double upper_limit = 0.0;
		double velocity_limit = 0.0;
		if (limit_xml) {
			try {
				std::string effort_str(limit_xml->Attribute("effort"));
			    torque_limit = boost::lexical_cast<double>(effort_str.c_str());
			    
			    std::string lower_str(limit_xml->Attribute("lower"));
			    lower_limit = boost::lexical_cast<double>(lower_str.c_str());
			    
			    std::string upper_str(limit_xml->Attribute("upper"));
			    upper_limit = boost::lexical_cast<double>(upper_str.c_str());
			    
			    std::string vel_str(limit_xml->Attribute("velocity"));
			    velocity_limit = boost::lexical_cast<double>(vel_str.c_str());
			}
			catch (boost::bad_lexical_cast &e) {
						    								    									
		    }
		}
		
		joint_torque_limits_.push_back(torque_limit);
		lower_joint_limits_.push_back(lower_limit);
		upper_joint_limits_.push_back(upper_limit);
		joint_velocity_limits_.push_back(velocity_limit);
		
		// Joint dynamics
		TiXmlElement *dyn_xml = joint_xml->FirstChildElement("dynamics");
		double damping = 0.0;
		if (dyn_xml) {
			std::string damping_str(dyn_xml->Attribute("damping"));
			damping = boost::lexical_cast<double>(damping_str.c_str());
		}
		
		joint_dampings_.push_back(damping);
		
		// Joint types
		std::string joint_type(joint_xml->Attribute("type"));
		joint_types_.push_back(joint_type);
		if (joint_type == "revolute") {
			active_joints_.push_back(joint_name);
			active_joint_origins_.push_back(origin);
			active_joint_axes_.push_back(joint_axis);
			active_joint_torque_limits_.push_back(torque_limit);
			
			active_lower_joint_limits_.push_back(lower_limit);
			active_upper_joint_limits_.push_back(upper_limit);
			active_joint_velocity_limits_.push_back(velocity_limit);			
		}		
	}
}

bool ManipulatorRobot::initLinks(TiXmlElement *robot_xml) {
	for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
	{
		
		shared::Link link;		
		link.active = false;
		//Link names
		std::string link_name(link_xml->Attribute("name"));
		link_names_.push_back(link_name);
		link.name = link_name;
		
		//Link dimensions
		std::vector<double> link_dimension;
		TiXmlElement *coll_xml = link_xml->FirstChildElement("collision");
		if (coll_xml) {
			active_link_names_.push_back(link_name);
			TiXmlElement *geom_xml = coll_xml->FirstChildElement("geometry");			
			TiXmlElement *dim_xml = geom_xml->FirstChildElement("box");			
		    const char* xyz_str = dim_xml->Attribute("size");		    
		    std::vector<std::string> pieces;					
		    boost::split( pieces, xyz_str, boost::is_any_of(" "));
		    for (unsigned int i = 0; i < pieces.size(); ++i) { 
			    if (pieces[i] != "") {
				    try {
					    link_dimension.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
				    }
				    catch (boost::bad_lexical_cast &e) {
							    									
				    }
			    }
		    }
		    if (link_dimension.size() != 3) {
		    	std::vector<double> ld({0.0, 0.0, 0.0});
		    	link_dimensions_.push_back(ld);
		    	for (auto &k: ld) {
		    		link.link_dimensions.push_back(k);
		    	}		    	
		    }
		    else {
		    	link_dimensions_.push_back(link_dimension);
		    	active_link_dimensions_.push_back(link_dimension);
		    	for (auto &k: link_dimension) {
		    		link.link_dimensions.push_back(k);
		    	}		    	
		    }
		    
		    std::vector<double> link_origin = process_origin_(coll_xml);
		    link_origins_.push_back(link_origin);		      
		    for (auto &k: link_origin) {
		    	link.origin.push_back(k);
		    }		    
		}
		else {
			std::vector<double> ld({0.0, 0.0, 0.0});
			link_dimensions_.push_back(ld);
			for (auto &k: ld) {
				link.link_dimensions.push_back(k);
			}
			
		}
		
		//Link inertia
		TiXmlElement *ine = link_xml->FirstChildElement("inertial");
		
		if (ine) {
			
			// Link masses
			active_links_.push_back(link_name);
			link.active = true;
			
			TiXmlElement *mass_xml = ine->FirstChildElement("mass");
			double mass = 0.0;
			if (mass_xml) {
				try {
					if (mass_xml->Attribute("value")) {
						mass = boost::lexical_cast<double>(mass_xml->Attribute("value"));
					}
				}
				catch (boost::bad_lexical_cast &e) {
					
				}
			}
			link_masses_.push_back(mass);
			link.mass = mass;
			
			//Inertia origins
			std::vector<double> inertia_origin = process_origin_(ine);
			link_inertia_origins_.push_back(inertia_origin);
			for (auto &k: inertia_origin) {
				link.inertia_origin.push_back(k);
			}
			
			//Inertia matrix
			std::vector<double> inertia_vals;
			double ixx = 0.0;
			double ixy = 0.0;
			double ixz = 0.0;
			double iyy = 0.0;
			double iyz = 0.0;
			double izz = 0.0;
			TiXmlElement *matr_xml = ine->FirstChildElement("inertia");
			if (matr_xml) {				
				try {
				    if (matr_xml->Attribute("ixx")) {
					    ixx = boost::lexical_cast<double>(matr_xml->Attribute("ixx"));					    
				    }
				    if (matr_xml->Attribute("ixy")) {
				        ixy = boost::lexical_cast<double>(matr_xml->Attribute("ixy"));
				    }
				    if (matr_xml->Attribute("ixz")) {
				    	ixz = boost::lexical_cast<double>(matr_xml->Attribute("ixz"));
				    }
				    if (matr_xml->Attribute("iyy")) {
				    	iyy = boost::lexical_cast<double>(matr_xml->Attribute("iyy"));
				    }
				    if (matr_xml->Attribute("iyz")) {
				    	iyz = boost::lexical_cast<double>(matr_xml->Attribute("iyz"));
				    }
				    if (matr_xml->Attribute("izz")) {
				    	izz = boost::lexical_cast<double>(matr_xml->Attribute("izz"));
				    }
				}
				catch (boost::bad_lexical_cast &e) {
									
				}
			}
				
			inertia_vals.push_back(ixx);
			inertia_vals.push_back(ixy);
			inertia_vals.push_back(ixz);
			inertia_vals.push_back(iyy);
			inertia_vals.push_back(iyz);
			inertia_vals.push_back(izz);
			link.inertials.push_back(ixx);
			link.inertials.push_back(ixy);
			link.inertials.push_back(ixz);
			link.inertials.push_back(iyy);
			link.inertials.push_back(iyz);
			link.inertials.push_back(izz);
			
			link_inertia_matrices_.push_back(inertia_vals);
		}
		else {			
			link_masses_.push_back(0.0);
			link.mass = 0.0;
			
			std::vector<double> origin({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
			link_inertia_origins_.push_back(origin);
			for (auto &k: origin) {
				link.inertia_origin.push_back(k);
			}
			
			std::vector<double> inert({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
			link_inertia_matrices_.push_back(inert);
			for (auto &k: inert) {
				link.inertials.push_back(k);
			}
		}
		
		links_.push_back(link);
	}
	
	
	
	return true;
}

std::vector<double> ManipulatorRobot::process_origin_(TiXmlElement *xml) {
	TiXmlElement *origin_xml = xml->FirstChildElement("origin");
	std::vector<double> origin;
	if (origin_xml) {				
		if (origin_xml->Attribute("xyz")) {
			const char* xyz_str = origin_xml->Attribute("xyz");
			const char* rpy_str = origin_xml->Attribute("rpy");
			std::vector<std::string> pieces;					
			boost::split( pieces, xyz_str, boost::is_any_of(" "));
			for (unsigned int i = 0; i < pieces.size(); ++i){
				if (pieces[i] != "") {
					try {
						origin.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
					}
					catch (boost::bad_lexical_cast &e) {
								
					}
				}						
			}
			
			pieces.clear();
			boost::split( pieces, rpy_str, boost::is_any_of(" "));
			for (unsigned int i = 0; i < pieces.size(); ++i){
				if (pieces[i] != "") {
					try {
						origin.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
					}
					catch (boost::bad_lexical_cast &e) {
											
					}
				}						
			}
		}				
	}
	if (origin.size() == 6) {
		return origin;
	}
	else {
		std::vector<double> orig({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
		return orig;
	}
}

ManipulatorRobot::ManipulatorRobot(std::string robot_file):
	Robot(robot_file),	
	links_(),
	joints_(),
	link_names_(),
	active_link_names_(),
	joint_names_(),
	joint_origins_(),
	link_origins_(),
	active_joint_origins_(),
	active_links_(),
	active_joints_(),
	joint_axes_(),
	active_joint_axes_(),
	joint_torque_limits_(),
	lower_joint_limits_(),
	upper_joint_limits_(),
	joint_velocity_limits_(),
	active_joint_velocity_limits_(),
	active_joint_torque_limits_(),	
	active_lower_joint_limits_(),
	active_upper_joint_limits_(),
	link_masses_(),
	link_inertia_origins_(),
	robot_state_(),	
	kinematics_(new Kinematics()),	
	rbdl_interface_(nullptr),
	goal_position_(),
	goal_radius_(0){
	
	propagator_ = std::make_shared<shared::ManipulatorPropagator>();
#ifdef USE_URDF	
	viewer_ = std::make_shared<shared::ViewerInterface>();
#endif
	TiXmlDocument xml_doc;
	xml_doc.LoadFile(robot_file);	
	TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
	initLinks(robot_xml);
	initJoints(robot_xml);
	
	std::vector<double> stateLowerLimits;
	std::vector<double> stateUpperLimits;
	getStateLimits(stateLowerLimits, stateUpperLimits);
	
	propagator_->setup(stateLowerLimits,
			           stateUpperLimits,			           
					   constraints_enforced_);
	kinematics_->setJointOrigins(joint_origins_);	
	kinematics_->setLinkDimensions(active_link_dimensions_);
	static_cast<shared::ManipulatorPropagator *>(propagator_.get())->getIntegrator()->setJointDamping(joint_dampings_);
}

void ManipulatorRobot::setNewtonModel() {
	rbdl_interface_ = std::make_shared<RBDLInterface>();
	rbdl_interface_->load_model(robot_file_);
	static_cast<shared::ManipulatorPropagator *>(propagator_.get())->getIntegrator()->setRBDLInterface(rbdl_interface_);
	rbdl_interface_->setViscous(joint_dampings_);
    rbdl_interface_->setPositionConstraints(lower_joint_limits_, upper_joint_limits_);    
}

void ManipulatorRobot::quatFromRPY(double &roll, double &pitch, double &yaw, std::vector<double> &quat) {
	double phi, the, psi;
	 
    phi = roll / 2.0;
	the = pitch / 2.0;
	psi = yaw / 2.0;
    double x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
	double y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
	double z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
	double w = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
	
	double s = sqrt(x * x +
	                y * y +
	                z * z +
	                w * w);
	
	if (s == 0.0) {
	    x = 0.0;
	    y = 0.0;
	    z = 0.0;
	    w = 1.0;
	}
	else {
	    x /= s;
	    y /= s;
	    z /= s;
	    w /= s;
	}
	
	quat.push_back(x);
	quat.push_back(y);
	quat.push_back(z);
	quat.push_back(w);
}

std::vector<std::shared_ptr<fcl::CollisionObject>> ManipulatorRobot::createRobotCollisionObjectsPy(const std::vector<double> &joint_angles) {
	std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects;	
	createRobotCollisionObjects(joint_angles, collision_objects);	
	return collision_objects;
}

std::vector<std::shared_ptr<fcl::CollisionObject>> 
ManipulatorRobot::createEndEffectorCollisionObjectPy(const std::vector<double> &joint_angles) {
	std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects;
	createEndEffectorCollisionObject(joint_angles, collision_objects);
	return collision_objects;
}

void ManipulatorRobot::setGoalArea(std::vector<double> &goal_position, double &goal_radius) {
	goal_position_.clear();
	for (size_t i = 0; i < goal_position.size(); i++) {
		goal_position_.push_back(goal_position[i]);
	}
	
	goal_radius_ = goal_radius;
}

void ManipulatorRobot::initCollisionObjects() {
	// Init the link collision objects
	std::vector<fcl::AABB> link_aabbs;
	for (size_t i = 0; i < active_link_dimensions_.size(); i++) {
	    /**link_aabbs.push_back(fcl::AABB(fcl::Vec3f(0.0, 
	        		                              -active_link_dimensions_[i][1] / 2.0, 
											      -active_link_dimensions_[i][2] / 2.0),
	                                   fcl::Vec3f(active_link_dimensions_[i][0], 
	                                    		  active_link_dimensions_[i][1] / 2.0, 
											      active_link_dimensions_[i][2] / 2.0)));*/
		link_aabbs.push_back(fcl::AABB(fcl::Vec3f(-active_link_dimensions_[i][0] / 2.0, 
			        		                      -active_link_dimensions_[i][1] / 2.0, 
											      -active_link_dimensions_[i][2] / 2.0),
			                           fcl::Vec3f(active_link_dimensions_[i][0] / 2.0, 
			                                      active_link_dimensions_[i][1] / 2.0, 
												  active_link_dimensions_[i][2] / 2.0)));
	}
	for (size_t i = 0; i < active_link_dimensions_.size(); i++) {
		fcl::Box* box = new fcl::Box();
		fcl::Transform3f box_tf;
		fcl::Transform3f trans;
		fcl::constructBox(link_aabbs[i], trans, *box, box_tf);
		collision_objects_.push_back(std::make_shared<fcl::CollisionObject>(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf));
	}
	
	// Init the end-effector collision object
	fcl::AABB aabb(fcl::Vec3f(0.0,
				              -active_link_dimensions_[active_link_dimensions_.size() - 1][1] / 2.0, 
			                  -active_link_dimensions_[active_link_dimensions_.size() - 1][2] / 2.0),
				   fcl::Vec3f(0.001,
						      active_link_dimensions_[active_link_dimensions_.size() - 1][1] / 2.0, 
						      active_link_dimensions_[active_link_dimensions_.size() - 1][2] / 2.0));
	fcl::Box* box = new fcl::Box();
    fcl::Transform3f box_tf;
    fcl::Transform3f trans;
    fcl::constructBox(aabb, trans, *box, box_tf);
    collision_objects_.push_back(std::make_shared<fcl::CollisionObject>(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf));
}

void ManipulatorRobot::createRobotCollisionObjects(const std::vector<double> &state, 
		                                           std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects) const {
	Eigen::MatrixXd res = Eigen::MatrixXd::Identity(4, 4);
	res(0, 3) = joint_origins_[0][0];
	res(1, 3) = joint_origins_[0][1];
	res(2, 3) = joint_origins_[0][2];
	
	for (size_t i = 0; i < state.size(); i++) {
		res = kinematics_->getPoseOfLinkN(state[i], res, i);
		fcl::Matrix3f trans_matrix(res(0,0), res(0,1), res(0,2),
				                   res(1,0), res(1,1), res(1,2),
								   res(2,0), res(2,1), res(2,2));
		fcl::Vec3f trans_vec(res(0,3), res(1,3), res(2,3));
		fcl::Transform3f trans(trans_matrix, trans_vec);
		fcl::AABB link_aabb(fcl::Vec3f(0.0, 
					        		   -active_link_dimensions_[i][1] / 2.0, 
				                       -active_link_dimensions_[i][2] / 2.0),
					        fcl::Vec3f(active_link_dimensions_[i][0], 
					                   active_link_dimensions_[i][1] / 2.0, 
			                           active_link_dimensions_[i][2] / 2.0));
		fcl::Box* box = new fcl::Box();  
		fcl::Transform3f box_tf;		
		fcl::constructBox(link_aabb, trans, *box, box_tf);		
		std::shared_ptr<fcl::CollisionObject> coll_obj = 
				std::make_shared<fcl::CollisionObject>(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf);
		collision_objects.push_back(coll_obj);
	}	
}

bool ManipulatorRobot::checkSelfCollision(std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects) const{	
	for (size_t i = 0; i < collision_objects.size(); i++) {
		if (i + 2 < collision_objects.size()) {
			for (size_t j = i + 2; j < collision_objects.size(); j++) {
				fcl::CollisionRequest request;
				fcl::CollisionResult result;
				fcl::collide(collision_objects[i].get(), 
							 collision_objects[j].get(),
							 request,
							 result);
				if (result.isCollision()) {			
					return true;
				}
			}
		}
	}
	
	return false;
}

void ManipulatorRobot::getLinearProcessMatrices(std::vector<double> &state, 
    	    			                        std::vector<double> &control, 
    	    			                        double &duration,
    	    			                        std::vector<Eigen::MatrixXd> &matrices) const {
	static_cast<shared::ManipulatorPropagator *>(propagator_.get())->getIntegrator()->getProcessMatrices(state, control, duration, matrices);
	matrices.push_back(Eigen::MatrixXd::Identity(getDOF() * 2, getDOF() * 2));
	matrices.push_back(Eigen::MatrixXd::Identity(getDOF() * 2, getDOF() * 2));
}
    	    
bool ManipulatorRobot::checkSelfCollision(const std::vector<double> &state) const {
	std::vector<std::shared_ptr<fcl::CollisionObject>> robot_collision_objects;
	createRobotCollisionObjects(state, robot_collision_objects);
	return checkSelfCollision(robot_collision_objects);
}

bool ManipulatorRobot::checkSelfCollisionPy(boost::python::list &ns) {
    std::vector<std::shared_ptr<fcl::CollisionObject>> robot_collision_objects;
    for (int i = 0; i < len(ns); ++i)
    {
        robot_collision_objects.push_back(boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[i]));
    }
    
    return checkSelfCollision(robot_collision_objects);
}

void ManipulatorRobot::createEndEffectorCollisionObject(const std::vector<double> &joint_angles,
    	    		std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects) {	
	const std::pair<fcl::Vec3f, fcl::Matrix3f> pose_ee = kinematics_->getPoseOfLinkN(joint_angles, active_link_dimensions_.size());
	fcl::Transform3f trans(pose_ee.second, pose_ee.first);
	fcl::Transform3f trans_res = trans * fcl::Transform3f(collision_objects_[collision_objects_.size() - 1]->getAABB().center());
	collision_objects_[collision_objects_.size() - 1]->setTransform(trans_res);
	collision_objects.push_back(collision_objects_[collision_objects_.size() - 1]);
}

void ManipulatorRobot::getPositionOfLinkN(const std::vector<double> &joint_angles, const int &n, std::vector<double> &position) {
	kinematics_->getPositionOfLinkN(joint_angles, n, position);
}
    	    
void ManipulatorRobot::getEndEffectorPosition(const std::vector<double> &joint_angles, std::vector<double> &end_effector_position) const {
	kinematics_->getEndEffectorPosition(joint_angles, end_effector_position);
}


/****************************************
 * Viewer functions
 */
#ifdef USE_URDF
void ManipulatorRobot::addPermanentViewerParticles(const std::vector<std::vector<double>> &particle_joint_values,
							            const std::vector<std::vector<double>> &particle_colors) {
	assert(particle_joint_values.size() == particle_colors.size() &&  
		   "Number of particles must be the same as number of colours!");
	viewer_->addPermanentParticles(particle_joint_values,
			                       particle_colors);	
}

void ManipulatorRobot::removePermanentViewerParticles() {
	viewer_->removePermanentParticles();
}

void ManipulatorRobot::updateViewerValues(const std::vector<double> &current_joint_values,
                               const std::vector<double> &current_joint_velocities,
							   const std::vector<std::vector<double>> &particle_joint_values,
							   const std::vector<std::vector<double>> &particle_colors) {		
	assert(particle_joint_values.size() == particle_colors.size() &&  
		   "Number of particles must be the same as number of colours!");
	// particle_color = {r, g, b, a}
	viewer_->updateRobotValues(current_joint_values, 
			                   current_joint_velocities,							  
							   particle_joint_values,
							   particle_colors,
							   nullptr);
}

void ManipulatorRobot::setViewerSize(int x, int y) {
	viewer_->setViewerSize(x, y);
}

void ManipulatorRobot::setViewerBackgroundColor(double r, double g, double b) {
	viewer_->setBackgroundColor(r, g, b);
}

void ManipulatorRobot::setViewerCameraTransform(std::vector<double> &rot, std::vector<double> &trans) {
	viewer_->setCameraTransform(rot, trans);
}
#endif


bool ManipulatorRobot::propagate_linear(std::vector<double> &current_state,
    	    		         std::vector<double> &control_input,
    	    		         std::vector<double> &control_error,
    	    		         double duration,
    	    		         std::vector<double> &result) {
	return static_cast<shared::ManipulatorPropagator *>(propagator_.get())->propagate_linear(current_state,
			                             control_input,
			                             control_error,
			                             duration,
			                             result);	
}

void ManipulatorRobot::setGravityConstant(double gravity_constant) {
	static_cast<shared::ManipulatorPropagator *>(propagator_.get())->getIntegrator()->setGravityConstant(gravity_constant);
	if (rbdl_interface_) {
		rbdl_interface_->setGravity(gravity_constant);
	}	
}

void ManipulatorRobot::setExternalForce(double f_x, 
		                     double f_y, 
							 double f_z,
							 double f_roll,
							 double f_pitch,
							 double f_yaw) {
	static_cast<shared::ManipulatorPropagator *>(propagator_.get())->getIntegrator()->setExternalForce(f_x, f_y, f_z, f_roll, f_pitch, f_yaw);	
}

void ManipulatorRobot::setAccelerationLimit(double accelerationLimit) {
	static_cast<shared::ManipulatorPropagator *>(propagator_.get())->getIntegrator()->setAccelerationLimit(accelerationLimit);
}

void ManipulatorRobot::getEndEffectorJacobian(const std::vector<double> &joint_angles, 
    	    		               std::vector<std::vector<double>> &ee_jacobian) {
	
	//std::vector<double> state;
	std::vector<double> state2;
	//for (auto &k: joint_angles) {
	//	state.push_back(k);		
	//}
	
	if (joint_angles.size() > getStateSpaceDimension() / 2) {
	    for (size_t i = 0; i < joint_angles.size() / 2; i++) {
		    state2.push_back(joint_angles[i]);
	    }
	}
	else {
		for (size_t i = 0; i < joint_angles.size(); i++) {
		    state2.push_back(joint_angles[i]);
		}
	}
	
	MatrixXd jacobian(6, getStateSpaceDimension() / 2);	
	kinematics_->getEEJacobian(state2, jacobian);
	for (size_t i = 0; i < jacobian.rows(); i++) {
		std::vector<double> row;
		for (size_t j = 0; j < jacobian.cols(); j++) {			
			row.push_back(jacobian(i, j));
		}		
		ee_jacobian.push_back(row);
	}
}

void ManipulatorRobot::getEndEffectorVelocity(std::vector<double> &state,
    	    		               std::vector<double> &ee_velocity) {
	MatrixXd j = static_cast<shared::ManipulatorPropagator *>(propagator_.get())->get_ee_jacobian(state);
	MatrixXd vel(state.size() / 2, 1);
	for (size_t i = 0; i < state.size() / 2; i++) {
		vel(i, 0) = state[i + state.size() / 2];
	}
	
	MatrixXd res = j * vel;
	ee_velocity.clear();
	for (size_t i = 0; i < 6; i++) { 
		ee_velocity.push_back(res(i, 0));
	}	
}

bool ManipulatorRobot::propagate_first_order(std::vector<double> &current_state,
                                  std::vector<double> &control_input,
                                  std::vector<double> &control_error,
		                          std::vector<double> &nominal_state,
		                          std::vector<double> &nominal_control,
                                  double simulation_step_size,
                                  double duration,
                                  std::vector<double> &result) {
	std::vector<double> current_joint_values;
	std::vector<double> current_joint_velocities;
		
	for (size_t i = 0; i < current_state.size() / 2; i++) {
		current_joint_values.push_back(current_state[i]);
		current_joint_velocities.push_back(current_state[i + current_state.size() / 2]);
	}
	
	return static_cast<shared::ManipulatorPropagator *>(propagator_.get())->propagate_nonlinear_first_order(current_joint_values,
				                                         current_joint_velocities,
				                                         control_input,
				                                         control_error,
														 nominal_state,
														 nominal_control,
				                                         simulation_step_size,
				                                         duration,
				                                         result);
}

bool ManipulatorRobot::propagate_second_order(std::vector<double> &current_state,
        std::vector<double> &control_input,
        std::vector<double> &control_error,
		std::vector<double> &nominal_state,
		std::vector<double> &nominal_control,
        double simulation_step_size,
        double duration,
        std::vector<double> &result) {
	std::vector<double> current_joint_values;
	std::vector<double> current_joint_velocities;
		
	for (size_t i = 0; i < current_state.size() / 2; i++) {
		current_joint_values.push_back(current_state[i]);
		current_joint_velocities.push_back(current_state[i + current_state.size() / 2]);
	}
	
	return static_cast<shared::ManipulatorPropagator *>(propagator_.get())->propagate_nonlinear_second_order(current_joint_values,
				                                         current_joint_velocities,
				                                         control_input,
				                                         control_error,
														 nominal_state,
														 nominal_control,
				                                         simulation_step_size,
				                                         duration,
				                                         result);
}

void ManipulatorRobot::setState(std::vector<double> &joint_values, std::vector<double> &joint_velocities) {
	robot_state_.joint_values = joint_values;
	robot_state_.joint_velocities = joint_velocities;
}

void ManipulatorRobot::getState(std::vector<double> &state) {
	for (auto &s: robot_state_.joint_values) {
		state.push_back(s);
	}
	
	for (auto &s: robot_state_.joint_velocities) {
		state.push_back(s);
	}
}

unsigned int ManipulatorRobot::get_link_index(std::string &link_name) {	
	for (size_t i = 0; i < link_names_.size(); i++) {
		if (link_name == link_names_[i]) {
			return i;
		}
	}
	
	return 0;
}

void ManipulatorRobot::getLinkNames(std::vector<std::string> &link_names) {
	for (auto &name: link_names_) {
		link_names.push_back(name);
	}	
}

void ManipulatorRobot::getJointNames(std::vector<std::string> &joint_names) {
	for (auto &name: joint_names_) {
		joint_names.push_back(name);
	}	
}

void ManipulatorRobot::getLinkMasses(std::vector<std::string> &link, std::vector<double> &link_masses) {
	int index = 0;
	for (size_t i = 0; i < link.size(); i++) {
		index = get_link_index(link[i]);
		link_masses.push_back(link_masses_[index]);
	}	
}

void ManipulatorRobot::getLinkInertias(std::vector<std::string> &link, std::vector<std::vector<double>> &inertias) {
	double index = 0;
	for (size_t i = 0; i < link.size(); i++) {
		index = get_link_index(link[i]);
		inertias.push_back(link_inertia_matrices_[index]);
	}
}

void ManipulatorRobot::getActiveLinkDimensions(std::vector<std::vector<double>> &dimensions) {
	for (auto &k: active_link_dimensions_) {
		dimensions.push_back(k);
	}
}

void ManipulatorRobot::getLinkDimension(std::vector<std::string> &link, std::vector<std::vector<double>> &dimension) {	
	int index = 0;
	for (size_t i = 0; i < link.size(); i++) {				
		index = get_link_index(link[i]);
		dimension.push_back(link_dimensions_[index]);
	}
}

void ManipulatorRobot::getLinkPose(std::vector<std::string> &link, std::vector<std::vector<double>> &pose) {
	int index = 0;
	for (size_t i = 0; i < link.size(); i++) {
		index = get_link_index(link[i]);		
		pose.push_back(link_origins_[index]);
	}
}

void ManipulatorRobot::getLinkInertialPose(std::vector<std::string> &link, std::vector<std::vector<double>> &pose) {
	int index = 0;
	for (size_t i = 0; i < link.size(); i++) {
		index = get_link_index(link[i]);
		pose.push_back(link_inertia_origins_[index]);
	}
}

void ManipulatorRobot::getActiveJoints(std::vector<std::string> &joints) const{
	for (auto &joint: active_joints_) {
		joints.push_back(joint);
	}
}

bool ManipulatorRobot::enforceConstraints(std::vector<double> &state) const {
	std::vector<double> lowerStateLimits; 
	std::vector<double> upperStateLimits;
	getStateLimits(lowerStateLimits, upperStateLimits);
	bool return_val = true;
	for (size_t i = 0; i < state.size() / 2; i++) {
		if (state[i] < lowerStateLimits[i]) {
			state[i] = lowerStateLimits[i];
			state[i + state.size() / 2] = 0.0;
			return_val = false;
		}
		else if (state[i] > upperStateLimits[i]) {
			state[i] = upperStateLimits[i];
			state[i + state.size() / 2] = 0.0;
			return_val = false;
		}
		
		if (state[i + state.size() / 2] < lowerStateLimits[i + state.size() / 2]) {
			state[i + state.size() / 2] = lowerStateLimits[i + state.size() / 2];
			return_val = false;
		}
		
		else if (state[i + state.size() / 2] > upperStateLimits[i + state.size() / 2]) {
			state[i + state.size() / 2] = upperStateLimits[i + state.size() / 2];
			return_val = false;
		}
	}
	
	return return_val;
}

void ManipulatorRobot::getStateLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const {
	std::vector<std::string> activeJoints;
	getActiveJoints(activeJoints);
	getJointLowerPositionLimits(activeJoints, lowerLimits);
	getJointUpperPositionLimits(activeJoints, upperLimits);	
	std::vector<double> velocityLimits;
	getJointVelocityLimits(activeJoints, velocityLimits);
	for (auto &k: velocityLimits) {
		lowerLimits.push_back(-k);
		upperLimits.push_back(k);
	}
}

void ManipulatorRobot::getControlLimits(std::vector<double> &lowerLimits, std::vector<double> &upperLimits) const {
	std::vector<std::string> activeJoints;
	getActiveJoints(activeJoints);
	getJointTorqueLimits(activeJoints, upperLimits);
	for (auto &k: upperLimits) {
		lowerLimits.push_back(-k);
	}
}

void ManipulatorRobot::getJointLowerPositionLimits(std::vector<std::string> &joints, std::vector<double> &joint_limits) const{
	int index = 0;
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joint_names_.size(); j++) {
			if (joints[i] == joint_names_[j]) {
				joint_limits.push_back(lower_joint_limits_[j]);
			}
		}
	}
}

void ManipulatorRobot::getJointUpperPositionLimits(std::vector<std::string> &joints, std::vector<double> &joint_limits) const{
	int index = 0;
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joint_names_.size(); j++) {
			if (joints[i] == joint_names_[j]) {
				joint_limits.push_back(upper_joint_limits_[j]);
			}
		}
	}
}

void ManipulatorRobot::getJointVelocityLimits(std::vector<std::string> &joints, std::vector<double> &joint_limits) const{
	int index = 0;
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joint_names_.size(); j++) {
			if (joints[i] == joint_names_[j]) {
				joint_limits.push_back(joint_velocity_limits_[j]);
			}
		}
	}
}

void ManipulatorRobot::getJointTorqueLimits(std::vector<std::string> &joints, std::vector<double> &joint_limits) const{
	int index = 0;
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joint_names_.size(); j++) {
			if (joints[i] == joint_names_[j]) {
				joint_limits.push_back(joint_torque_limits_[j]);
			}
		}
	}
}

void ManipulatorRobot::getJointDamping(std::vector<std::string> &joints, std::vector<double> &damping) {
	int index = 0;
	for (size_t i = 0; i < joints.size(); i++) {
	    for (size_t j = 0; j < joint_names_.size(); j++) {
		    if (joints[i] == joint_names_[j]) {
				damping.push_back(joint_dampings_[j]);
			}
		}
	}
}

void ManipulatorRobot::getJointType(std::vector<std::string> &joint, std::vector<std::string> &type) {
	int index = 0;
	for (size_t i = 0; i < joint.size(); i++) {
		for (size_t j = 0; j < joint_names_.size(); j++) {
			if (joint[i] == joint_names_[j]) {
				type.push_back(joint_types_[j]);
			}
		}
	}	
}

void ManipulatorRobot::getJointOrigin(std::vector<std::string> &joints, std::vector<std::vector<double>> &origins) {
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joint_names_.size(); j++) {
			if (joints[i] == joint_names_[j]) {
				origins.push_back(joint_origins_[j]);
			}
		}
	}
}

void ManipulatorRobot::getJointAxis(std::vector<std::string> &joints, std::vector<std::vector<int>> &axis) {
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joint_names_.size(); j++) {
			if (joints[i] == joint_names_[j]) { 
				axis.push_back(joint_axes_[j]);
			}
		}
	}
}

int ManipulatorRobot::getStateSpaceDimension() {
	return active_joints_.size() * 2;
}

int ManipulatorRobot::getControlSpaceDimension() {
	return active_joints_.size();
}

int ManipulatorRobot::getDOF() const {	
	return active_joints_.size();
}

bool ManipulatorRobot::isTerminal(std::vector<double> &state) const {
	std::vector<double> joint_angles;
	std::vector<double> end_effector_position;
	for (size_t i = 0; i < joint_angles.size() / 2; i++) {
		joint_angles.push_back(state[i]);
	}
	
	getEndEffectorPosition(joint_angles, end_effector_position);
	double dist = 0.0;
	for (size_t i = 0; i < end_effector_position.size(); i++) {
		dist += std::pow(end_effector_position[i] - goal_position_[i], 2);
	}
	
	dist = std::sqrt(dist);
	if (dist < goal_radius_) {
		return true;
	}
	
	return false;
}

std::vector<double> ManipulatorRobot::getProcessMatrices(std::vector<double> &x, 
                                              std::vector<double> &rho, 
				                              double t_e) {
	return static_cast<shared::ManipulatorPropagator *>(propagator_.get())->getIntegrator()->getProcessMatricesVec(x, rho, t_e);
}

std::shared_ptr<shared::Robot> makeManipulatorRobot(std::string model_file) {
	std::shared_ptr<shared::Robot> robot_ptr = std::make_shared<ManipulatorRobot>(model_file);
	return robot_ptr;
}

BOOST_PYTHON_MODULE(librobots) {
    using namespace boost::python;
    
    boost::python::type_info info= boost::python::type_id<std::vector<double>>();
    const boost::python::converter::registration* reg_double = boost::python::converter::registry::query(info);
    if (reg_double == NULL || (*reg_double).m_to_python == NULL)  {
    	class_<std::vector<double> > ("v_double")
    	    .def(vector_indexing_suite<std::vector<double> >());
    }
    
    info = boost::python::type_id<std::vector<int>>();
    const boost::python::converter::registration* reg_int = boost::python::converter::registry::query(info);
    if (reg_int == NULL || (*reg_int).m_to_python == NULL)  {    
    	class_<std::vector<int> > ("v_int")
    	    .def(vector_indexing_suite<std::vector<int> >());
    }
    
    info = boost::python::type_id<std::vector<std::vector<double>>>();
    const boost::python::converter::registration* reg_v2double = boost::python::converter::registry::query(info);
    if (reg_v2double == NULL || (*reg_v2double).m_to_python == NULL)  {  
    	class_<std::vector<std::vector<double> > > ("v2_double")
    	    .def(vector_indexing_suite<std::vector<std::vector<double> > >());
    }
    
    info = boost::python::type_id<std::vector<std::vector<int>>>();
    const boost::python::converter::registration* reg_v2int = boost::python::converter::registry::query(info);
    if (reg_v2int == NULL || (*reg_v2int).m_to_python == NULL)  {    
    	class_<std::vector<std::vector<int> > > ("v2_int")
    	    .def(vector_indexing_suite<std::vector<std::vector<int> > >());
    }
    
    info = boost::python::type_id<std::vector<std::string>>();
    const boost::python::converter::registration* reg_vstring = boost::python::converter::registry::query(info);
    if (reg_vstring == NULL || (*reg_vstring).m_to_python == NULL)  {    	
    	class_<std::vector<std::string> > ("v_string")
    	    .def(vector_indexing_suite<std::vector<std::string> >());
    }
    	
    class_<fcl::OBB>("OBB");
    class_<fcl::CollisionObject>("CollisionObject", init<const boost::shared_ptr<fcl::CollisionGeometry>, const fcl::Transform3f>());
    to_python_converter<std::vector<fcl::OBB, std::allocator<fcl::OBB> >, VecToList<fcl::OBB> >();
    to_python_converter<std::vector<fcl::CollisionObject, std::allocator<fcl::CollisionObject> >, VecToList<fcl::CollisionObject> >();
    to_python_converter<std::vector<std::shared_ptr<fcl::CollisionObject>, std::allocator<std::shared_ptr<fcl::CollisionObject>> >, 
        VecToList<std::shared_ptr<fcl::CollisionObject>> >();
    register_ptr_to_python<std::shared_ptr<fcl::CollisionObject>>();
    
    class_<Robot, std::shared_ptr<Robot>, boost::noncopyable>("Robot", no_init);
    
    class_<RobotWrapper, boost::noncopyable>("Robot", init<std::string>())
    		.def("getDOF", &RobotWrapper::getDOF)
    		.def("getStateSpaceDimension", &RobotWrapper::getStateSpaceDimension)
    		.def("getControlSpaceDimension", &RobotWrapper::getControlSpaceDimension)
    ;
    class_<ManipulatorRobot, boost::shared_ptr<ManipulatorRobot>, bases<Robot>>("ManipulatorRobot", init<std::string>())
                        .def("getLinkNames", &ManipulatorRobot::getLinkNames)
                        .def("getLinkDimension", &ManipulatorRobot::getLinkDimension)
                        .def("getActiveLinkDimensions", &ManipulatorRobot::getActiveLinkDimensions)
                        .def("getLinkMasses", &ManipulatorRobot::getLinkMasses)
                        .def("getLinkPose", &ManipulatorRobot::getLinkPose)
                        .def("getLinkInertialPose", &ManipulatorRobot::getLinkInertialPose)
                        .def("getLinkInertias", &ManipulatorRobot::getLinkInertias)
                        .def("getJointNames", &ManipulatorRobot::getJointNames)
                        .def("getActiveJoints", &ManipulatorRobot::getActiveJoints)
                        .def("getJointType", &ManipulatorRobot::getJointType)
						.def("getJointDamping", &ManipulatorRobot::getJointDamping)
                        .def("getJointOrigin", &ManipulatorRobot::getJointOrigin)
                        .def("getJointAxis", &ManipulatorRobot::getJointAxis)
                        .def("propagate", &ManipulatorRobot::propagateState)
						.def("propagate_first_order", &ManipulatorRobot::propagate_first_order)
						.def("propagate_second_order", &ManipulatorRobot::propagate_second_order)
                        //.def("createRobotCollisionStructures", &Robot::createRobotCollisionStructuresPy)
                        .def("createRobotCollisionObjects", &ManipulatorRobot::createRobotCollisionObjectsPy)
						.def("createEndEffectorCollisionObject", &ManipulatorRobot::createEndEffectorCollisionObjectPy)
                        .def("getEndEffectorPosition", &ManipulatorRobot::getEndEffectorPosition)
                        .def("getStateSpaceDimension", &ManipulatorRobot::getStateSpaceDimension)
                        .def("getControlSpaceDimension", &ManipulatorRobot::getControlSpaceDimension)
                        .def("getDOF", &ManipulatorRobot::getDOF)
						.def("getJointLowerPositionLimits", &ManipulatorRobot::getJointLowerPositionLimits)
						.def("getJointUpperPositionLimits", &ManipulatorRobot::getJointUpperPositionLimits)
						.def("getJointVelocityLimits", &ManipulatorRobot::getJointVelocityLimits)
						.def("getJointTorqueLimits", &ManipulatorRobot::getJointTorqueLimits)
						.def("enforceConstraints", &ManipulatorRobot::enforceConstraints)
						.def("constraintsEnforced", &ManipulatorRobot::constraintsEnforced)
						.def("setGravityConstant", &ManipulatorRobot::setGravityConstant)
						.def("setExternalForce", &ManipulatorRobot::setExternalForce)
						.def("setAccelerationLimit", &ManipulatorRobot::setAccelerationLimit)
						.def("getEndEffectorVelocity", &ManipulatorRobot::getEndEffectorVelocity)
						.def("getProcessMatrices", &ManipulatorRobot::getProcessMatrices)						
						.def("getEndEffectorJacobian", &ManipulatorRobot::getEndEffectorJacobian)						
						.def("setNewtonModel", &ManipulatorRobot::setNewtonModel)
						.def("checkSelfCollision", &ManipulatorRobot::checkSelfCollisionPy)
#ifdef USE_URDF
						.def("setupViewer", &ManipulatorRobot::setupViewer)						
						.def("updateViewerValues", &ManipulatorRobot::updateViewerValues)
						.def("setViewerSize", &ManipulatorRobot::setViewerSize)
						.def("setViewerBackgroundColor", &ManipulatorRobot::setViewerBackgroundColor)
					    .def("setViewerCameraTransform", &ManipulatorRobot::setViewerCameraTransform)
					    .def("addPermanentViewerParticles", &ManipulatorRobot::addPermanentViewerParticles)
					    .def("removePermanentViewerParticles", &ManipulatorRobot::removePermanentViewerParticles)
#endif
                        //.def("setup", &Integrate::setup)                        
    ;
    
    /**def("makeManipulatorRobot", &makeManipulatorRobot,
    	return_value_policy<manage_new_object>());*/
    def("makeManipulatorRobot", &makeManipulatorRobot);
}

}
