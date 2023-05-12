#!/usr/bin/env python3


'''
C++ code generation for Modified Denavit-Hartenberg parameters and URDF files

python dh_code.py file.yml (from yaml file)
python dh_code.py file.urdf base effector (from URDF file)

author: Olivier Kermorgant, ICube Laboratory 
'''

import yaml
from lxml import etree
import sys, os
import sympy as sp
from sympy.parsing.sympy_parser import parse_expr
from pylab import pi, array, norm
import re
from multiprocessing import Pool
import argparse
from subprocess import check_output

# Utility functions and variables
X = sp.Matrix([1,0,0]).reshape(3,1)
Y = sp.Matrix([0,1,0]).reshape(3,1)
Z = sp.Matrix([0,0,1]).reshape(3,1)
Z4 = sp.Matrix([0,0,0,1]).reshape(4,1)
cst_symb = {}

def sk(u):
    return sp.Matrix([[0,-u[2],u[1]],[u[2],0,-u[0]],[-u[1],u[0],0]])

def Rot(theta,u):
    R = sp.cos(theta)*sp.eye(3) + sp.sin(theta)*sk(u) + (1-sp.cos(theta))*(u*u.transpose())
    return sp.Matrix(R)

def Rxyz(rpy):
    '''
    Rotation matrix in X - Y - Z convention, so multiply the matrices in reversed order
    '''
    return Rot(rpy[2],Z)*Rot(rpy[1],Y)*Rot(rpy[0],X)

def Homogeneous(t, R):
    '''
    Homogeneous frame transformation matrix from translation t and rotation R
    '''
    M = simp_matrix((R.row_join(t)).col_join(sp.Matrix([[0,0,0,1]])))
    return M

class Bunch(object):
    def __init__(self, adict):
        self.__dict__.update(adict)

def load_yaml(filename):
    '''
    Loads the given yaml file content with DH parameters. Builds the corresponding data.
    '''
    with open(filename) as f:
        robot = yaml.safe_load(f)
    robot['keys'] = [k for k in robot]
    robot = Bunch(robot)  

    # get ordering
    if 'notation' in robot.keys:
            iAlpha = robot.notation.index('alpha')
            iA = robot.notation.index('a')
            iR = robot.notation.index('r')
            iTheta = robot.notation.index('theta')
    else:
            iAlpha = 0
            iA = 1
            iR = 2
            iTheta = 3
            
    # change into symbolic
    print('')
    print('Building intermediary matrices...')
    
    prism = []     # True if prismatic
    T = []         # relative T(i-1,i) 
    u = []         # joint axis
    fM0 = None
    eMw = None
    
    for q,joint in robot.joint.items():
        this_prism = None
        if type(joint[iR]) == str:
            if 'q' in joint[iR]:
                this_prism = True
        if type(joint[iTheta]) == str:
            if 'q' in joint[iTheta]:
                this_prism = False
        if this_prism is not None:
            prism.append(this_prism)
        for i in range(4):
            if type(joint[i]) == str:
                joint[i] = parse_expr(joint[i])
        if q == 'f':
            fM0 = Homogeneous(joint[iA]*X, Rot(joint[iAlpha],X)) * Homogeneous(joint[iR]*Z, Rot(joint[iTheta],Z))
        elif q == 'e':
            eMw = Homogeneous(joint[iA]*X, Rot(joint[iAlpha],X)) * Homogeneous(joint[iR]*Z, Rot(joint[iTheta],Z))
        else:
            # transformation matrix
            T.append(Homogeneous(joint[iA]*X, Rot(joint[iAlpha],X)) * Homogeneous(joint[iR]*Z, Rot(joint[iTheta],Z)))
            # joint axis, always Z in DH convention
            u.append(Z)
    return T, u, prism, fM0, eMw


def simp_rpy(rpy):
    '''
    Converts floating angle values to fractions of pi
    '''
    rpy = [parse_expr(v) for v in rpy]
    for i in range(3):
        for k in range(-12,13):
            if abs(rpy[i] - k*pi/12.) < 1e-5:
                if k != 0:
                    print('  changing', rpy[i], 'to', sp.simplify(k*sp.pi/12))
                rpy[i] = str(sp.simplify(k*sp.pi/12))
                if rpy[i] == '0':
                    rpy[i] = 0
                break
    return rpy

def simp_axis(u):
    '''
    Convert x y z axis to real 1 and 0
    '''
    u = [parse_expr(v) for v in u]
    for i in range(3):
        for v in (-1,0,1):
            if abs(u[i]-v) < 1e-5:
                u[i] = v
    return sp.Matrix(u).reshape(3,1)

def simp_val(val, idx):
    '''
    Convert numeric x y z to symbolic ones
    '''
    global cst_symb
    val = [parse_expr(v) for v in val]
    for i in range(3):
        if abs(val[i]) < 1e-5:
            val[i] = 0
        else:
            s = 'abd'[i] + str(idx)
            cst_symb[s] = val[i]
            val[i] = sp.Symbol(s)
    return sp.Matrix(val).reshape(len(val),1)

def load_urdf(filename):
    # reads as URDF or XACRO depending on file name
    if filename.endswith('.urdf'):
        with open(filename) as f:
            return f.read()
    else:
        urdf = check_output(('xacro ' + filename).split(), encoding='UTF8')
        if urdf[0] == 'w':
            return check_output(('xacro ' + filename).split(), encoding='UTF8')
        return urdf
        
def parse_urdf(filename, base_frame, ee_frame, use_joint_names = False):
    '''
    Parse the URDF file to extract the geometrical parameters between given frames
    '''
    robot = etree.fromstring(load_urdf(filename).encode())
        
    # find all joints
    parents = []
    children = []
    all_joints = robot.findall('joint')
    for joint in all_joints:
        parents.append(joint.find('parent').get('link'))
        children.append(joint.find('child').get('link'))
        
    if base_frame not in parents:
        print('Could not find', base_frame, 'in parent link list')
        print('Known parents: ' + " ".join(set(parents)))
        sys.exit(0)
    if ee_frame not in children:
        print('Could not find', ee_frame, 'in children link list')
        print('Known children: ' + " ".join(set(children)))
        sys.exit(0)
    # find path from base link to effector link
    joint_path = []
    cur_link = ee_frame
    while cur_link != base_frame:
        try:
            i = children.index(cur_link)
        except:
            print('Could not find', cur_link, 'in link list')
            sys.exit(0)
        
        if i in joint_path:
            print('Error: passed 2 times through', cur_link)
            sys.exit(0)
            
        joint_path.append(i)
        cur_link = parents[i]        
    joint_path.reverse()     
    
    # build robot geometry
    n = 0
    M = sp.eye(4)
    T = []
    prism = []
    u = []
    all_q = []
    parent = base_frame
    fM0 = wMe = None
    last_moving = 0
    joints = [all_joints[i] for i in joint_path]
    
    for k, joint in enumerate(joints):
        
        # get this transform
        try:
            xyz = simp_val(joint.find('origin').get('xyz').split(' '), k+1)
        except:
            xyz = sp.zeros(3,1)
        try:
            rpy = simp_rpy(joint.find('origin').get('rpy').split(' '))
        except:
            rpy = sp.zeros(3,1)
        
        Mi = Homogeneous(xyz, Rxyz(rpy))
        # get quaternion
        
        #print 'from', joint.find('parent').get('link'), 'to', child
        #print Mi
        
        if joint.get('type') != 'fixed':            
            last_moving = k
            if n == 0 and k != 0:
                # there were some fixed joints before this one, build a constant matrix fM0
                fM0 = M
                M = Mi
                print('Constant matrix fM0 between', base_frame, 'and', joints[k-1].find('child').get('link'))
            else:
                M = M*Mi
            n += 1
            #print 'joint', n, ': from', parent, 'to', child
            #print M
            # prismatic?
            prism.append(joint.get('type') == 'prismatic')
            # axis
            ax = array([float(v) for v in joint.find('axis').get('xyz').split(' ')])
            ax = ax/norm(ax)
            u.append(simp_axis([str(v) for v in ax]))
            # Transform matrix
            if use_joint_names:
                q = sp.Symbol(joint.get('name'))
            else:
                q = sp.Symbol('q%i'%n)
            all_q.append(q)
            if prism[-1]:
                T.append(M * Homogeneous(q*u[-1], Rot(0, X)))
            else:
                T.append(M * Homogeneous(0*X, Rot(q, u[-1])))
            # reset M for next joint
            M = sp.eye(4)
        else:
            M = M*Mi
            
    if joint.get('type') == 'fixed':
        # we finished on some fixed links
        nonI = False
        for i in range(3):
            for j in range(4):
                if (i == j and M[i,j] != 1) or (i != j and M[i,j] != 0):
                    nonI = True
            
        if nonI:
            wMe = M      
            print('Constant matrix wMe between', joints[last_moving].find('child').get('link'), 'and', ee_frame)
    return T, u, prism, fM0, wMe, all_q


# human sorting
def human_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text 
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
    l.sort( key=alphanum_key )

def simp_matrix(M):
    '''
    simplify matrix for old versions of sympy
    '''
    for i in range(M.rows):
        for j in range(M.cols):       
            M[i,j] = sp.trigsimp(M[i,j])
            # check for these strange small numbers
            '''
            s = str(M[i,j])
            almost0 = False
            for k in range(20, 50):
                if 'e-' + str(k) in s:
                    almost0 = True
                    break
            
            if almost0:
                for k in range(20, 50):
                    while 'e-' + str(k) in s:
                        m = s.find('e-' + str(k))
                        n = m
                        while s[n] != '.':
                            n -= 1
                        s = s.replace(s[n-1:m+4], '0')
                M[i,j] = sp.trigsimp(parse_expr(s))
            '''            
    return M    

def compute_Ji(joint_prism, u0, p0, i):
    '''
    Compute the i-eth column of the Jacobian (used for multiprocessing)
    '''
    if joint_prism[i] == None:  # fixed joint
        return sp.zeros(6,0)
    
    if joint_prism[i]:
        # prismatic joint: v = qdot.u and w = 0
        Jv = simp_matrix(u0[i])
        Jw = sp.Matrix([[0,0,0]]).reshape(3,1)
    else:
        # revolute joint: v = [qdot.u]x p and w = qdot.u
        Jv = simp_matrix(sk(u0[i])*(p0[-1]-p0[i]))
        Jw = simp_matrix(u0[i])
    print('   J_%i' % (i+1))
    return (i, Jv.col_join(Jw))    # register this column as column i
    #return Jv.col_join(Jw)


def replaceFctQ(s, cDef, cUse, q = 'q', q_vector = True):
    '''
    Replace cos and sin functions of q_i with precomputed constants
    '''
    fctList = ('cos', 'sin')
    pmDict = {'+':'', '-':'m'}
    defCount = len(cDef)
    # replace with all expressions already found
    for sf in cUse:
        s = s.replace(sf, cUse[sf])
        
    # look for new expressions
    for fct in fctList:
        while True:
            pos = s.find(fct)
            if pos != -1:
                end = pos + s[pos:].find(')')                   
                sf = s[pos:end+1]                                       # sf = cos(q1 + q2 - q3)
                expr = s[pos+len(fct)+1:end].split(' ')                 # expr = [q1,+,q2,-,q3]
                cUse[sf] = fct[0]                
                sUse = fct + '('
                for v in expr:
                    if 'q' in v:
                        cUse[sf] += v[1:]
                        i = int(v[1:])
                        if q_vector:
                            sUse += f'{q}[{i-1}]'
                        else:
                            sUse += f'{q}{i}'
                    else:
                        cUse[sf] += pmDict[v]                
                        sUse += v
                sUse += ')'                                             # cos(q[0]+q[1]-q[2])
                cDef[sf] = 'const double %s = %s;' % (cUse[sf], sUse)   # const c1p2m3 = cos(q[0]+q[1]-q[2]);
                s = s.replace(sf, cUse[sf])
            else:
                break
            
    # other occurences of qi
    for i in range(100):
        s = s.replace('q%i' % (i+1), '%s[%i]' % (q, i))
    return s.replace('00000000000000', '').replace('0000000000000', ''), cDef, cUse

def exportCpp(M, s='M', q = 'q', col_offset = 0, q_vector = True):
        '''
        Writes the C++ code corresponding to a given matrix
        '''
        cDef={}
        cUse={}
        M_lines = []

        # write each element
        sRows = ''
        sCols = ''
        for i in range(M.rows):
                if M.rows > 1:
                        sRows = '[' + str(i) + ']'
                for j in range(M.cols):
                        if M.cols > 1:
                                sCols = '[' + str(j+col_offset) + ']'
                        ms, cDef, cUse = replaceFctQ(str(sp.N(M[i,j])), cDef, cUse, q, q_vector)
                        M_lines.append(s + sRows + sCols + ' = ' + ms + ';')
                        
        # print definitions
        cDef = list(cDef.values())
        human_sort(cDef)
        for line in cDef:
            print('   ',line)
        # print matrix content
        for line in M_lines:
            print('   ', line)
            
def ComputeDK_J(T, u, prism, comp_all = False):
     # get number of joints
    dof = len(T)
    
    # Transform matrices
    print('')
    print('Building direct geometric model...')
    T0 = []     # absolute T(0,i)
    for i in range(dof):
        if len(T0) == 0:
            T0.append(T[i])
        else:
            T0.append(simp_matrix(T0[-1]*T[i]))
        print('  T %i/0' % (i+1))
        
    # Jacobian   
    # Rotation of each frame to go to frame 0
    print('')
    print('Building kinematic model...')
    
    R0 = [M[:3,:3] for M in T0]
    # joint axis expressed in frame 0
    u0 = [R0[i]*u[i] for i in range(dof)] 
    
    all_J = []
    
    if comp_all:
        ee_J = range(1,dof+1)
    else:
        ee_J = [dof]
    for ee in ee_J:
        # origin of each frame expressed in frame 0
        p0 = [T0[i][:3,3] for i in range(ee)]
                    
        # build Jacobian
        # Sympy + multithreading bug = argument is not an mpz
        #pool = Pool()
        #results = []
        #for i in range(ee):
            ## add this column to pool
            #results.append(pool.apply_async(compute_Ji, args=(prism, u0, p0, i)))
        #iJ = [result.get() for result in results]
        #pool.close()  
        
        iJ = [compute_Ji(prism, u0, p0, i) for i in range(ee)]

        Js = sp.zeros(6, ee)

        for i in range(ee):
            for k,iJi in iJ:
                if k == i:
                    Js[:,i] = iJi
                    #Js = Js.row_join(iJi)
        all_J.append(Js.copy())
    print('')
  
    return T0, all_J

def latex_print(M):
    s = sp.latex(M)
    s = s.replace('\\cos', 'c').replace('\\sin', 's')
    n = max([i for i in range(1,10) if '_{'+str(i)+'}' in s])
    single = '{{\\left(q_{{{}}} \\right)}}'
    double = '{{\\left(q_{{{}}} + q_{{{}}} \\right)}}'
    for i1 in range(1, n+1):
        s = s.replace(single.format(i1), '_{}'.format(i1))
        for i2 in range(1,n):
            s = s.replace(double.format(i1,i2), '_{{{}{}}}'.format(i1,i2))
    print(s)
    

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.description = 'A module to generate C++ code from URDF or Yaml (DH) file.'

    # files
    parser.add_argument('files', metavar='file', type=str, nargs='+', help='File (and base and end-effector frames for URDF)')

    parser.add_argument('-q', metavar='q', help='How the joint vector appears in the code',default='q')
    parser.add_argument('-T', metavar='M', help='How the pose matrix appears in the code',default='M')
    parser.add_argument('-J', metavar='J', help='How the Jacobian matrix appears in the code',default='J') 
    parser.add_argument('--all_J', action='store_true', help='Computes the Jacobian of all frames',default=False)
    parser.add_argument('--only-fixed', action='store_true', help='Only computes the fixed matrices, before and after the arm',default=False)
    parser.add_argument('--display', action='store_true', help='Prints the full model',default=False)
    parser.add_argument('--wrist', action='store_true', help='Prints the model of the wrist to help computing inverse geometry',default=False)
    parser.add_argument('--latex', action='store_true', help='Prints direct model and Jacobian in Latex style',default=False)
    parser.add_argument('--only-DGM', action='store_true', help='Only DGM',default=False)
    parser.add_argument('--fMe', action='store_true', help='Prints the full fMe transform',default=False)
    parser.add_argument('--eJe', action='store_true', help='Prints the eJe Jacobian',default=False)
    
    args = parser.parse_args()

    def pretty_print(M):
        if args.latex:
            latex_print(M)
        else:
            sp.pretty_print(M)

    # check robot description file
    if not os.path.lexists(args.files[0]):
            print('File', args.files[0], 'does not exist')
            sys.exit(0)
    fM0 = wMe = None
    
    # load into symbolic
    print('')
    print('Building intermediary matrices...')
    if args.files[0].split('.')[-1] in ('urdf','xacro'):
        if len(args.files) == 3:
            T, u, prism, fM0, wMe, all_q = parse_urdf(args.files[0], args.files[1], args.files[2])
        else:
            print('Not enough arguments for URDF parsing - frames needed')
            sys.exit(0)
    elif args.files[0][-4:] == 'yaml' or args.files[0][-3:] == 'yml':
        T, u, prism, fM0, wMe = load_yaml(args.files[0])
    else:
        print('Unknown file type', args.files[0])

    # get number of joints
    dof = len(T)

    fixed_M = ((wMe, 'wMe','end-effector'), (fM0,'fM0','base frame'))
    for M,symbol,title in fixed_M:
        if M != None:
            print('')
            print(f'Building {symbol} code...')
            print('')
            print(f'    // Generated {title} code')
            exportCpp(M, symbol)
            print(f'    // End of {title} code')

    if args.only_fixed:
        sys.exit(0)

    
    if args.only_DGM:
    
        # Transform matrices
        print('')
        print('Building direct geometric model...')
        T0 = []     # absolute T(0,i)
        for i in range(dof):
            if len(T0) == 0:
                T0.append(T[i])
            else:
                T0.append(simp_matrix(T0[-1]*T[i]))
            print('  T %i/0' % (i+1))
        print('')
        print('Building pose C++ code...')
        print('')
        print('    // Generated pose code')
        exportCpp(T0[-1], args.T)
        print('    // End of pose code')
        sys.exit(0)
        
    
    else:
        
        # Do the computation
        T0, all_J = ComputeDK_J(T, u, prism, args.all_J)

        if not args.display:
            print('')
            print('Building pose C++ code...')
            print('')
            print('    // Generated pose code')
            exportCpp(T0[-1], args.T)
            print('    // End of pose code')

            print('')
            print('Building Jacobian C++ code...')
            if args.all_J:
                for i,Js in enumerate(all_J):
                    print('')
                    print('    // Generated Jacobian code to link %i'% (i+1))
                    exportCpp(Js, args.J + str(i+1), args.q)
                    print('    // End of Jacobian code to link %i'% (i+1))
            else:
                print('')
                print('    // Generated Jacobian code')
                exportCpp(all_J[-1], args.J, args.q)
                print('    // End of Jacobian code')

        if len(cst_symb):
            print('\n//Model constants')
            lines = []
            for key in cst_symb:
                line = 'const double {} = {};'.format(key, cst_symb[key])
                while line[-2] == '0':
                    line = line[:-2] + ';'
                lines.append(line)
            print('\n'.join(sorted(lines)))
            print('// End of constants')
        
    if args.display:
        print('\n\nFull model from root to wrist frame:')
        print('\nTranslation')
        pretty_print(T0[-1][:3,3])
        print('\nRotation')
        pretty_print(T0[-1][:3,:3])
        
                
    if args.wrist and dof == 6:        
        print('\n\nDecomposing DGM with regards to frame 3:')
        
        print('\nTranslation from root to wrist frame 0T6 (should only depend on q1 q2 q3):\n')
        pretty_print(T0[-1][:3,3])
        
        print('\n\nRotation 3R6 from frame 3 to wrist frame (should only depend on q4 q5 q6):\n')
        R36 = simp_matrix(T[3][:3,:3] * T[4][:3,:3] * T[5][:3,:3])
        pretty_print(R36)
            
        print('\n\nCode for the rotation 0R3 from root frame to frame 3:\n')
        exportCpp(T0[2][:3,:3], 'R03', q_vector=False)

    I4 = sp.eye(4)
    
    fMe = (fM0 if fM0 is not None else I4) * T0[-1] * (wMe if wMe is not None else I4)
    if args.fMe or args.latex:
        print('\n\nModel from base to end-effector frame fMe')
    else:
        print('\n\nModel from base to end-effector frame fMe for q=0')
        for n in range(len(T0)):
            fMe = fMe.subs(sp.Symbol(f'q{n}'), 0)

    fMe = simp_matrix(fMe)
    pretty_print(fMe)
                
    if args.all_J:
        pretty_print(all_J[-1])


    if args.eJe:
        O3 = sp.zeros(3,3)
        def from_blocks(A,B,C,D):
            return (A.row_join(B)).col_join(C.row_join(D))
        
        eMw = wMe.inv() if wMe is not None else I4
        eTw = eMw[:3,[3]]
        eRw = eMw[:3,:3]
        eWw = from_blocks(eRw, sk(eTw)*eRw, O3, eRw)
        wRf = ((fM0 if fM0 is not None else I4) * T0[-1])[:3,:3].T
        wRRf = from_blocks(wRf, O3, O3, wRf)
        eJe = eWw * wRRf * all_J[-1]
        print('\neJe:')
        exportCpp(eJe, 'J')
