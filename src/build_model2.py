import os
import sys
import argparse
from sympy import *
import sympybotics
from urdf_parser_py import urdf

class ModelParser:
  def __init__(self, file, header_src, imple_src):
    self.build_model(file)
    print "Calculate link jacobians"
    linkJacobians, baseToCOMTransformations = self.calcLinkJacobians2()
    g_ = symbols("g_")
    g = Matrix([0.0, 0.0, g_]).T
    print "Calculate inertia matrix"
    M = self.calcManipulatorInertiaMatrix(linkJacobians)
    print "Calculate coriolis matrix"
    C = self.calcCentrifugalMatrix(M)
    print "Calculate normal forces"
    N = self.calcNormalForces(g, baseToCOMTransformations)
    numLinks = len(self.link_names)    
    print "invert inertia matrix"
    M_inv = self.inertia_inverse(M, symbolic=True)
    print "get dynamic model"    
    f = self.get_dynamic_model(M, M_inv, C, N, self.q, self.qdot, self.rho, self.zeta)
    print "Calculate partial derivatives"  
    A, B, V = self.partial_derivatives2(f)
    print "Calc first order derivatives of observation function"
    H, W = self.calc_observation_derivatives(baseToCOMTransformations)
    print "cleaning cpp code..."    
    
    self.clean_cpp_code(header_src, imple_src)
    self.gen_cpp_code2(f, "F0", header_src, imple_src)
    #self.gen_cpp_code2(A, "A0", header_src, imple_src)
    #self.gen_cpp_code2(B, "B0", header_src, imple_src)
    #self.gen_cpp_code2(V, "V0", header_src, imple_src)
    self.gen_cpp_code2(M, "M0", header_src, imple_src)
    self.gen_cpp_code2(H, "H0", header_src, imple_src)
    self.gen_cpp_code2(W, "W0", header_src, imple_src)
    print "done"
  
  def calcNormalForces(self, g, baseToCOMTransformations):     
    Ocs = []    
    for i in xrange(len(baseToCOMTransformations)):
	print baseToCOMTransformations[i]
	Oc = Matrix([baseToCOMTransformations[i][0, 3],
	             baseToCOMTransformations[i][1, 3],
	             baseToCOMTransformations[i][2, 3]])
	Ocs.append(Oc)
    
    V = 0.0
    link_masses = self.link_masses[1:len(self.link_masses)-1]    
    for i in xrange(len(link_masses)):
      el = g[2] * Ocs[i][2] * link_masses[i] 
      print "el " + str(el)
      V = V + el
      V = trigsimp(V)     
    q = [self.q[i] for i in xrange(len(self.q) - 1)]
    VDiff = trigsimp(Matrix([V]).jacobian(q).T)
    return VDiff
    print VDiff 
    print VDiff.shape
    N = Matrix([[trigsimp(diff(V, self.q[i]).doit())] for i in xrange(len(self.q) - 1)])
    print N;sleep
    qdot = self.qdot[0:len(self.qdot) - 1]
    '''
    The joint friction forces
    '''
    K = N + Matrix([[self.viscous[i] * qdot[i]] for i in xrange(len(qdot))])    
    return K  
    
  def calcCentrifugalMatrix(self, M):    
    C = zeros(M.shape[0], M.shape[1])
    q = self.q[0:len(self.q) - 1]
    qdot = self.qdot[0:len(self.qdot) - 1]    
    for i in xrange(M.shape[0]):
      for j in xrange(M.shape[1]):	
	for k in xrange(M.shape[0]):
	  C[i, j] += (trigsimp(diff(M[i, j], q[k])) + 
	              trigsimp(diff(M[i, k], q[j])) + 
	              trigsimp(diff(M[k, j], q[i]))) * qdot[k]
	C[i, j] *= 0.5
    '''for k in xrange(M.shape[0]):
      for j in xrange(M.shape[1]):
	for i in xrange(M.shape[0]):
	  C[k, j] += (trigsimp(diff(M[k, j], q[i])) +
	              trigsimp(diff(M[k, i], q[j])) +
	              trigsimp(diff(M[i, j], q[k]))) * qdot[i]
	C[k,j] *= 0.5'''        
    return C
    
  def calcManipulatorInertiaMatrix(self, linkJacobians):    
    Ms = []
    for i in xrange(len(self.joint_origins) - 1):
      M = eye(6)
      for j in xrange(3):
	M[j, j] = self.link_masses[i + 1]
      M[3, 3] = self.Is[i+1][0]
      M[4, 4] = self.Is[i+1][1]
      M[5, 5] = self.Is[i+1][2]
      Ms.append(M)
    M = zeros(len(linkJacobians), len(linkJacobians))
    for i in xrange(len(linkJacobians)):
      M += trigsimp(linkJacobians[i].T * Ms[i] * linkJacobians[i])
    return trigsimp(M)

  def getJointRotation(self, jointNumber):      
      jointAxis = self.joint_axis[jointNumber]
      print "jointNumber " + str(jointNumber)
      print "ja " + str(jointAxis)
      rotation = 0
      if (jointAxis[0] == 1.0):	  
	  rotation = self.rotateX(self.q[jointNumber])
      elif (jointAxis[1] == 1.0):	 
	  rotation = self.rotateY(self.q[jointNumber])
      elif (jointAxis[2] == 1.0):
	  
	  rotation = self.rotateZ(self.q[jointNumber])
      
      return rotation
       
  def calcLinkJacobians2(self):    
    baseToJointTransformations = []
    baseToCOMTransformations = []
    currentTrans = self.translateXYZ(0.0, 0.0, 0.0)    
    
    for i in xrange(len(self.joint_origins) - 1):
        jointRotation = 0
	if (i == 0):
	    jointRotation = self.rotateZ(0)
	else:	    
	    jointRotation = self.getJointRotation(i-1)	
	trans = self.translateXYZ(self.joint_origins[i][0], self.joint_origins[i][1], self.joint_origins[i][2])	
	rotX = self.rotateX(self.joint_origins[i][3])
	#rotY = self.rotateY(self.joint_origins[i][4])
	#rotZ = self.rotateZ(self.joint_origins[i][5])	
	currentTrans *= jointRotation	
	currentTrans *= trans	
	currentTrans *= rotX
	#currentTrans *= rotY
	#currentTrans *= rotZ	
	currentTrans = trigsimp(currentTrans)	
	baseToJointTransformations.append(currentTrans)    
    for i in xrange(len(self.joint_origins) - 1):
      jointRotation = self.getJointRotation(i)      
      inertialPose = self.inertial_poses[i+1]
      inertialTranslation = self.translateXYZ(inertialPose[0], inertialPose[1], inertialPose[2])      
      trans = baseToJointTransformations[i] * jointRotation * inertialTranslation
      trans = trigsimp(trans)
      baseToCOMTransformations.append(trans)    
    linkJacobians = []
    for i in xrange(len(self.joint_origins) - 1):
	J = zeros(6, len(self.joint_origins) - 1)
	Oc = trigsimp(Matrix([baseToCOMTransformations[i].col(3)[k] for k in xrange(3)]).T)
	for j in xrange(i + 1):
	  Oj = trigsimp(Matrix([baseToJointTransformations[j].col(3)[k] for k in xrange(3)]).T)	  
	  Zj = trigsimp(Matrix([baseToJointTransformations[j].col(2)[k] for k in xrange(3)]).T)
	  upper = Zj.cross(Oc - Oj)
	  lower = Zj
	  for k in xrange(3):
	    J[k, j] = trigsimp(upper[k])
	    J[k+3, j] = trigsimp(lower[k])
	linkJacobians.append(J)	
    return linkJacobians, baseToCOMTransformations
    
  def calcLinkJacobians(self):
    baseToJointTransformations = []
    currentTrans = 0
    for i in xrange(len(self.joint_origins) - 1):
      dh = 0
      if (i == 0):
	currentTrans = self.dh(0, self.joint_origins[i][2], 0, self.joint_origins[i][3])	
      else:
	currentTrans = currentTrans * self.dh(self.q[i - 1], 0, self.joint_origins[i][0], self.joint_origins[i][3])
      baseToJointTransformations.append(currentTrans)
    baseToCOMTransformations = []    
    for i in xrange(len(self.joint_origins) - 1):      
      trans = baseToJointTransformations[i] * self.dh(self.q[i], 0, self.joint_origins[i + 1][0] / 2.0, 0)
      baseToCOMTransformations.append(trans)
    linkJacobians = []
    for i in xrange(len(self.joint_origins) - 1):
	J = zeros(6, len(self.joint_origins) - 1)
	Oc = trigsimp(Matrix([baseToCOMTransformations[i].col(3)[k] for k in xrange(3)]).T)
	for j in xrange(i + 1):
	  Oj = trigsimp(Matrix([baseToJointTransformations[j].col(3)[k] for k in xrange(3)]).T)	  
	  Zj = trigsimp(Matrix([baseToJointTransformations[j].col(2)[k] for k in xrange(3)]).T)
	  upper = Zj.cross(Oc - Oj)
	  lower = Zj
	  for k in xrange(3):
	    J[k, j] = trigsimp(upper[k])
	    J[k+3, j] = trigsimp(lower[k])
	linkJacobians.append(J)
    return linkJacobians, baseToCOMTransformations

  def translateXYZ(self, x, y, z):
      return Matrix([[1.0, 0.0, 0.0, x],
		     [0.0, 1.0, 0.0, y],
		     [0.0, 0.0, 1.0, z],
		     [0.0, 0.0, 0.0, 1.0]])

  def rotateX(self, theta):
      return Matrix([[1.0, 0.0, 0.0, 0.0],
		     [0.0, cos(theta), -sin(theta), 0.0],
		     [0.0, sin(theta), cos(theta), 0.0],
		     [0.0, 0.0, 0.0, 1.0]])
  
  def rotateY(self, theta):
      return Matrix([[cos(theta), 0.0, sin(theta), 0.0],
		     [0.0, 1.0, 0.0, 0.0],
		     [-sin(theta), 0.0, cos(theta), 0.0],
		     [0.0, 0.0, 0.0, 1.0]])
  
  def rotateZ(self, theta):
      return Matrix([[cos(theta), -sin(theta), 0.0, 0.0],
		     [sin(theta), cos(theta), 0.0, 0.0],
		     [0.0, 0.0, 1.0, 0.0],
		     [0.0, 0.0, 0.0, 1.0]])
    
    
  def dh(self, theta, d, a, alpha):
        return Matrix([[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
                       [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
                       [0.0, sin(alpha), cos(alpha), d],
                       [0.0, 0.0, 0.0, 1.0]])
    
  def inertia_inverse(self, M, symbolic=False):
        if symbolic:
            M_inv = Matrix.zeros(M.shape[0], M.shape[1])
            for i in xrange(M.shape[0]):
                for j in xrange(M.shape[1]):
                    strr = "M_inv_(" + str(i) + ", " + str(j) + ")" 
                    s = Symbol(strr)
                    M_inv[i, j] = s
            return M_inv
        else:
            return M.inv()
    
  def calc_observation_derivatives(self, baseToCOMTransformations):
        ee_transformation = baseToCOMTransformations[-1] * self.transformation(self.joint_origins[1][0] / 2, 0.0, 0.0)
	g_funct = [trigsimp(ee_transformation[0, 3]), 
	           trigsimp(ee_transformation[1, 3]), 
	           trigsimp(ee_transformation[2, 3])]	
	for i in xrange(len(self.qdot) - 1):
	    g_funct.append(self.qdot[i])
	g_funct = Matrix(g_funct)
	g_funct = g_funct
	etas = [symbols("eta_[" + str(i) + "]") for i in xrange(g_funct.shape[0])]	
	
	for i in xrange(len(etas)):
	    g_funct[i, 0] = g_funct[i, 0] + etas[i]
	x = [self.q[i] for i in xrange(len(self.q) - 1)]
	x.extend([self.qdot[i] for i in xrange(len(self.qdot) - 1)])
	H = trigsimp(g_funct.jacobian([x[i] for i in xrange(len(x))]))
	W = trigsimp(g_funct.jacobian([etas[i] for i in xrange(len(etas))]))	
	H = simplify(H)
	H = nsimplify(H, tolerance=1e-4) 
	return H, W
    
  def transformation(self, x, y, z):
    t = Matrix([[1.0, 0.0, 0.0, x],
		[0.0, 1.0, 0.0, y],
		[0.0, 0.0, 1.0, z],
		[0.0, 0.0, 0.0, 1.0]])
    return t
    
  def partial_derivatives2(self, f):        
        A1 = f.jacobian([self.q[i] for i in xrange(len(self.q) - 1)])
        A2 = f.jacobian([self.qdot[i] for i in xrange(len(self.qdot) - 1)])
        B =  f.jacobian([self.rho[i] for i in xrange(len(self.rho) - 1)])
        C =  f.jacobian([self.zeta[i] for i in xrange(len(self.zeta) - 1)])
        A = A1.row_join(A2)        
        return A, B, C    
    
  def get_dynamic_model(self, M, M_inv, C, N, thetas, dot_thetas, rs, zetas):             
        #print "time to invert: " + str(time.time() - t0)
        Dotthetas = Matrix([[dot_thetas[i]] for i in xrange(len(dot_thetas) - 1)])       
        Rs = Matrix([[rs[i]] for i in xrange(len(rs) - 1)])        
        Zetas = Matrix([[zetas[i]] for i in xrange(len(zetas) - 1)])        
        print "Constructing 2nd-order ODE"
        m_upper = Matrix([[Dotthetas[i]] for i in xrange(len(Dotthetas))])
        m_lower = 0
        '''if self.simplifying:
            m_lower = trigsimp(-M_inv * trigsimp(C * Dotthetas + N) + M_inv * Rs)           
        else:'''
        m_lower = M_inv * ((Rs + Zetas) - C * Dotthetas - N)
        h = m_upper.col_join(m_lower)        
        return h

  def parse_urdf(self, file):
	  r = urdf.Robot.from_xml_string(file)
	  #robot = Robot(xml_file)
	  self.link_names = [link.name for link in r.links]
	  self.joint_names = [r.joints[i].name for i in xrange(len(r.joints))]
	  self.joint_types = [joint.type for joint in r.joints]
	  self.joint_origins = [Matrix([[joint.origin.xyz[0]],
					[joint.origin.xyz[1]],
					[joint.origin.xyz[2]],
					[joint.origin.rpy[0]],
					[joint.origin.rpy[1]],
					[joint.origin.rpy[2]]]) for joint in r.joints]
	  for i in xrange(len(self.joint_origins)):
	    self.joint_origins[i][3] = nsimplify(self.joint_origins[i][3], [pi], tolerance=0.001)
	    self.joint_origins[i][4] = nsimplify(self.joint_origins[i][4], [pi], tolerance=0.001)
	    self.joint_origins[i][5] = nsimplify(self.joint_origins[i][5], [pi], tolerance=0.001)
	    
	  self.joint_axis = [Matrix([[joint.axis[0]],
				    [joint.axis[1]],
				    [joint.axis[2]]]) for joint in r.joints]
	  self.viscous = [symbols("viscous_[" + str(i) + "]") for i in xrange(len(r.joints) - 1)]
	  print "==================="
	  
	  
	  self.q = []
	  self.qdot = []
	  self.qstar = []
	  self.qdotstar = []
	  self.rho = []
	  self.rhostar = []
	  self.zeta = []
	  self.zetastar = []
	  for i in xrange(len(self.joint_names)):                     
	      
	      symb_string_q = "x[" + str(i) + "]"
	      symb_string_q_dot = "x[" + str(i + len(self.joint_names) - 1) + "]"
	      symb_string_q_star = "xstar[" + str(i) + "]"
	      symb_string_q_dot_star = "xstar[" + str(i + len(self.joint_names) - 1) + "]"
	      symb_string_r = "rho[" + str(i) + "]"
	      symb_string_r_star = "rhostar[" + str(i) + "]" 
	      symb_zeta = "zeta[" + str(i) + "]"
	      symb_zeta_star = "zetastar[" + str(i) + "]"          
		  
		  
	      self.q.append(symbols(symb_string_q))
	      self.qdot.append(symbols(symb_string_q_dot))
	      self.rho.append(symbols(symb_string_r))
	      self.qstar.append(symbols(symb_string_q_star))
	      self.qdotstar.append(symbols(symb_string_q_dot_star))
	      self.rhostar.append(symbols(symb_string_r_star))
	      self.zeta.append(symbols(symb_zeta))
	      self.zetastar.append(symbols(symb_zeta_star))           
	  
	  self.inertial_poses = []
	  self.link_masses = []
	  self.link_inertias = []
	  self.link_dimensions = []
	  self.Is = []
	  for link in r.links:
	      if link.inertial != None:		  
		  self.inertial_poses.append([link.inertial.origin.xyz[0], 
					      link.inertial.origin.xyz[1],
					      link.inertial.origin.xyz[2],
					      link.inertial.origin.rpy[0],
					      link.inertial.origin.rpy[1],
					      link.inertial.origin.rpy[2]])
		  self.link_masses.append(link.inertial.mass)
		  self.link_inertias.append(Matrix(link.inertial.inertia.to_matrix()))
		  self.Is.append([self.link_inertias[-1][i, i] for i in xrange(3)])
	      else:
		  self.inertial_poses.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		  self.link_masses.append(0.0)
		  self.link_inertias.append(Matrix([[0.0 for j in xrange(3)] for i in xrange(3)]))
		  self.Is.append([0.0 for i in xrange(3)])


  def build_model(self, modelFile):
    if (modelFile == None):
      print "Model is none"
      return
    print("Parsing model " + modelFile)
    self.parse_urdf(modelFile)
    
  def gen_cpp_code2(self, Matr, name, header_src, imple_src):        
        lines = list(open(imple_src, 'r'))
        lines_header = list(open(header_src, 'r'))
        temp_lines = []
        if Matr.shape[1] != 1:
            temp_lines.append("MatrixXd m(" + str(Matr.shape[0]) + ", " + str(Matr.shape[1]) + "); \n")
        else:
            temp_lines.append("VectorXd m(" + str(Matr.shape[0]) + "); \n")        
        for i in xrange(Matr.shape[0]):
            for j in xrange(Matr.shape[1]):
                temp_lines.append("m(" + str(i) + ", " + str(j) + ") = " + str(ccode(Matr[i, j])) + "; \n")
        temp_lines.append("return m; \n")
        idx1 = -1
        idx2 = -1
        breaking = False    
        for i in xrange(len(lines)):
            if "Integrate::get" + name + "(const state_type &x, const state_type &rho, const state_type &zeta) const{" in lines[i]:                
                idx1 = i + 1               
                breaking = True
            elif "}" in lines[i]:
                idx2 = i - 1
                if breaking:
                    break        
        if idx1 == -1:            
            temp_lines.insert(0, "MatrixXd Integrate::get" + name + "(const state_type &x, const state_type &rho, const state_type &zeta) const{ \n") 
            temp_lines.append("\n")
            temp_lines.append("} \n \n")                     
            lines[len(lines) - 2:len(lines) - 1] = temp_lines            
            
            temp_lines_header = []
            idx = -1
            for i in xrange(len(lines_header)):
                if "private:" in lines_header[i]:                    
                    idx = i
            temp_lines_header.append("MatrixXd get" + str(name) + "(const state_type &x, const state_type &rho, const state_type &zeta) const; \n")
            lines_header[idx+1:idx+1] = temp_lines_header
                
        else:                  
            del lines[idx1:idx2]
            idx = -1
            for i in xrange(len(lines)):
                if "Integrate::get" + name in lines[i]:
                    idx = i        
            lines[idx:idx] = temp_lines           
        os.remove(imple_src)
        os.remove(header_src)
        with open(imple_src, 'a+') as f:
            for line in lines:
                f.write(line)
        with open(header_src, 'a+') as f:
            for line in lines_header:
                f.write(line)
                
  def clean_cpp_code2(self, header_src, imple_src):
        lines = list(open(imple_src, 'r'))
        lines_header = list(open(header_src, 'r'))
        tmp_lines = []
        idx_pairs = []
        
        idx1 = -1
        idx2 = -1
        breaking = False
        for i in xrange(len(lines)):
	    if ("MatrixXd Integrate::getA" in lines[i] or
	        "MatrixXd Integrate::getB" in lines[i] or
	        "MatrixXd Integrate::getV" in lines[i] or
	        "MatrixXd Integrate::getF" in lines[i] or
	        "MatrixXd Integrate::getM" in lines[i] or
	        "MatrixXd Integrate::getC" in lines[i] or
	        "MatrixXd Integrate::getN" in lines[i] or
		"MatrixXd Integrate::getH" in lines[i] or
                "MatrixXd Integrate::getW" in lines[i]):            
                idx1 = i                
                breaking = True
            if "}" in lines[i] and breaking:
                idx_pairs.append((idx1, i))
                idx1 = -1
                breaking = False               
        for i in xrange(len(lines)):
            app = True
            for j in xrange(len(idx_pairs)):
                if i >= idx_pairs[j][0] and i <= idx_pairs[j][1]:
                    app = False
                    break                
            if app:
                tmp_lines.append(lines[i])
        os.remove(imple_src)        
        with open(imple_src, 'a+') as f:
            for line in tmp_lines:
                f.write(line)
                
        tmp_lines = []
        idxs = []
        for i in xrange(len(lines_header)):
	    if ("MatrixXd Integrate::getA" in lines[i] or
	        "MatrixXd Integrate::getB" in lines[i] or
	        "MatrixXd Integrate::getV" in lines[i] or
	        "MatrixXd Integrate::getF" in lines[i] or
	        "MatrixXd Integrate::getM" in lines[i] or
	        "MatrixXd Integrate::getC" in lines[i] or
	        "MatrixXd Integrate::getN" in lines[i] or
		"MatrixXd Integrate::getH" in lines[i] or
                "MatrixXd Integrate::getW" in lines[i]):            
                idxs.append(i)
        for i in xrange(len(lines_header)):
            app = True
            for j in xrange(len(idxs)):
                if i == idxs[j]:
                    app = False
            if app:
                tmp_lines.append(lines_header[i])
                
        os.remove(header_src)        
        with open(header_src, 'a+') as f:
            for line in tmp_lines:
                f.write(line)
                
        lines = list(open(imple_src, 'r'))
        tmp_lines = []
        idx1 = -1
        idx2 = -1
        breaking = False
        for i in xrange(len(lines)):
            if "void Integrate::setupSteadyStates() const {" in lines[i]:
                idx1 = i + 1
                breaking = True
            elif "std::pair<Integrate::AB_funct, std::pair<Integrate::AB_funct, Integrate::AB_funct>> Integrate::getClosestSteadyStateFunctions" in lines[i]:
                idx2 = i - 3                
                if breaking:
                    break                     
        del lines[idx1:idx2]        
        os.remove(imple_src)        
        with open(imple_src, 'a+') as f:
            for line in lines:               
                f.write(line)
    
    
  def clean_cpp_code(self, header_src, imple_src):
        lines = list(open(imple_src, 'r'))
        lines_header = list(open(header_src, 'r'))
        tmp_lines = []
        idx_pairs = []
        
        idx1 = -1
        idx2 = -1
        breaking = False
        for i in xrange(len(lines)):
	    if ("MatrixXd Integrate::getA" in lines[i] or 
                "MatrixXd Integrate::getB" in lines[i] or 
                "MatrixXd Integrate::getV" in lines[i] or
                "MatrixXd Integrate::getF" in lines[i] or
                "MatrixXd Integrate::getM" in lines[i] or
                "MatrixXd Integrate::getC" in lines[i] or
                "MatrixXd Integrate::getN" in lines[i] or
                "MatrixXd Integrate::getH" in lines[i] or
                "MatrixXd Integrate::getW" in lines[i] or
                "MatrixXd Integrate::getSec" in lines[i] or
                "MatrixXd Integrate::getFirst" in lines[i] or
                "MatrixXd Integrate::getEEJacobian" in lines[i] or
                "MatrixXd Integrate::getMInv" in lines[i]):            
                idx1 = i                
                breaking = True
            if "}" in lines[i] and breaking:
                idx_pairs.append((idx1, i))
                idx1 = -1
                breaking = False               
        for i in xrange(len(lines)):
            app = True
            for j in xrange(len(idx_pairs)):
                if i >= idx_pairs[j][0] and i <= idx_pairs[j][1]:
                    app = False
                    break                
            if app:
                tmp_lines.append(lines[i])
        os.remove(imple_src)        
        with open(imple_src, 'a+') as f:
            for line in tmp_lines:
                f.write(line)
                
        tmp_lines = []
        idxs = []
        for i in xrange(len(lines_header)):
	    if ("MatrixXd getA" in lines_header[i] or 
                "MatrixXd getB" in lines_header[i] or 
                "MatrixXd getV" in lines_header[i] or
                "MatrixXd getF" in lines_header[i] or
                "MatrixXd getM" in lines_header[i] or
                "MatrixXd getC" in lines_header[i] or
                "MatrixXd getN" in lines_header[i] or
                "MatrixXd getH" in lines_header[i] or
                "MatrixXd getW" in lines_header[i] or
                "MatrixXd getSec" in lines_header[i] or
                "MatrixXd getFirst" in lines_header[i] or
                "MatrixXd getEEJacobian" in lines_header[i] or
                "MatrixXd getMInv" in lines[i]):            
                idxs.append(i)
        for i in xrange(len(lines_header)):
            app = True
            for j in xrange(len(idxs)):
                if i == idxs[j]:
                    app = False
            if app:
                tmp_lines.append(lines_header[i])
                
        os.remove(header_src)        
        with open(header_src, 'a+') as f:
            for line in tmp_lines:
                f.write(line)
                
        lines = list(open(imple_src, 'r'))
        tmp_lines = []
        idx1 = -1
        idx2 = -1
        breaking = False
        for i in xrange(len(lines)):
            if "void Integrate::setupSteadyStates() const {" in lines[i]:
                idx1 = i + 1
                breaking = True
            elif "std::pair<Integrate::AB_funct, std::pair<Integrate::AB_funct, Integrate::AB_funct>> Integrate::getClosestSteadyStateFunctions" in lines[i]:
                idx2 = i - 3                
                if breaking:
                    break                     
        del lines[idx1:idx2]        
        os.remove(imple_src)        
        with open(imple_src, 'a+') as f:
            for line in lines:               
                f.write(line)
  

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Dynamic model generator.')
  parser.add_argument('file', type=argparse.FileType('r'), nargs='?', default=None, help='File to load. Use - for stdin')
  parser.add_argument("-he", "--header", help="Path to the robot header file")
  parser.add_argument("-s", "--src", help="Path to the robot source file")
  
  args = parser.parse_args()  
  if (args.file == None):
    print "File is none"
    sys.exit(0)
  if (args.header == None):
    print "No header provided"
    sys.exit(0)
  if (args.src == None):
    print "No source provided"
    sys.exit(0)
  xml_string = args.file.read()
  ModelParser(xml_string, args.header, args.src)