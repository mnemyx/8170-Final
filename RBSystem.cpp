/*
 *
 *  RBSystem.h
 *  GBG - Proj #5 - Rigid Bodies
 *
 */

#include "RBSystem.h"

// help function
Matrix3x3 star(const Vector3d &omega) {
    Matrix3x3 w_star;

    w_star.set(0, -omega.z, omega.y, omega.z, 0, -omega.x, -omega.y, omega.x, 0);

    return w_star;
}

RBSystem::RBSystem(int nbods) {
    nbodies = nbods;
    rblist = new RBody[nbodies];

    for(int i = 0; i < nbodies; i++)
        rblist[i].setRBI(i);

    Y.setSize(nbodies * STATE_SIZE);
    Ydot.setSize(nbodies * STATE_SIZE);
}

RBSystem::~RBSystem() {
    delete []rblist;
}

void RBSystem::setParams(double m[], double width[], double height[], double depth[], int type[], double d1[], double d2[], double d3[], Vector4d c[]) {
    //cout << "nbodies -- setParams(): " << nbodies << endl;
    for(int i = 0; i < nbodies; i++) {
        //cout << "width[" << i << "]: " << width[i] << endl;
        //cout << "height[" << i << "]: " << height[i] << endl;
        //cout << "depth[" << i << "]: " << depth[i] << endl;
        rblist[i].setParams(m[i], width[i], height[i], depth[i], type[i], d1[i], d2[i], d3[i]);
        rblist[i].setColor(c[i]);
    }
}

void RBSystem::setEnv(Vector3d g, Vector3d w, double v) {
    Env.G = g;
    Env.W = w;
    Env.Vis = v;
}

void RBSystem::initializeState(Vector3d x0[], Quaternion q[], Vector3d v0[], Vector3d omega0[]) {
    for(int i = 0; i < nbodies; i++) {
        rblist[i].initICs(x0[i], q[i], v0[i], omega0[i]);
        //rblist[i].print();
    }
}

void XtoState(Vector3d &x, Quaternion &q, Vector3d &p, Vector3d &l, const StateVector X, const int rbi) {
    int i = rbi * 13;

    x.set(X[i++],X[i++],X[i++]);
    q.set(X[i++],X[i++],X[i++],X[i++]);
    p.set(X[i++],X[i++],X[i++]);
    l.set(X[i++],X[i++], X[i++]);
}

void StatetoX(const Vector3d x, const Quaternion q, const Vector3d p, const Vector3d l, StateVector &X, const int rbi) {
    int i = rbi * 13;

    X[i++] = x.x;
    X[i++] = x.y;
    X[i++] = x.z;
    X[i++] = q.q.w;
    X[i++] = q.q.x;
    X[i++] = q.q.y;
    X[i++] = q.q.z;
    X[i++] = p.x;
    X[i++] = p.y;
    X[i++] = p.z;
    X[i++] = l.x;
    X[i++] = l.y;
    X[i++] = l.z;
}

StateVector dynamics(const StateVector &X, double t, double dt, const int nbodies, const RBody &rb, const Environment &Env) {
    Vector3d x, p, l;
    Quaternion q;
    StateVector newXdot(nbodies * STATE_SIZE);
    Vector3d F, T, V, Q;
    Vector3d fg, fs, fd;
    Vector3d ts, tg, td;
    Vector3d w;
    Matrix3x3 r, iinv;
    Quaternion wq, qdot;

    //cout << "X prior to XtoState() in dynamics() line 180: " << endl;
    //X.print();
    //cout << endl;

    XtoState(x, q, p, l, X, rb.getrbi());

    //cout << "x: " << x << endl;
    //cout << "q: " << q << endl;
    //cout << "p: " << p << endl;
    //cout << "l: " << l << endl;

    // calc velocity
    V = p / rb.getM();
    //cout << "M: " << rb.getM() << endl;
    //cout << "V: " << V << endl;
    // calc rate of change of q:
    r = q.normalize().rotation();
    iinv = r * rb.getIbodyinv() * r.transpose();

    //cout << "iinv: \n";
    //iinv.print();

    w = iinv * l;
    wq.set(w);
    //cout << endl << "w: " << w << endl;
    //cout << endl << "wq: " << wq << endl;

    Q = 0.5 * wq * q;
    //cout << "Q: " << Q << endl;
    // calc force
    //fg = rb.getM() * Env.G;

    if (Env.W.x == 0 && Env.W.y == 0 && Env.W.z == 0)
        fg = (Env.G - Env.Vis * V ) * rb.getM();
    else
        fg = (Env.G + Env.Vis * (Env.W - V)) * rb.getM();

    //Vector3d p1 = rb.getvertex(spi);

    //fs = - (sp.GetK() / rb.getM()) * ((p1 - sp.GetP0()).norm() - sp.GetL0()) * (p1 - sp.GetP0()).normalize();
    //fd = - (sp.GetD() / rb.getM()) * ((V * (p1 - sp.GetP0()).normalize()) * (p1 - sp.GetP0()).normalize());
    //cout << "fs: " << fs << endl;

    //fs = -sp.GetK() * ((p1 - sp.GetP0()).norm() - sp.GetL0()) * (p1 - sp.GetP0()).normalize();

    //fd = -sp.GetD() * ((V * (p1 - sp.GetP0()).normalize()) * (p1 - sp.GetP0()).normalize());

    F = fg;// + fs + fd;

    // calc torque
    //    ts = (p1 - x).norm() % (fs);
    //    td = (p1 - x).norm() % (fd);

    //ts = (p1 - x) % fs;

    //td = (p1 - x) % fd;

    //T = ts + td;
    T.set(0, 0, 0);

    StatetoX(V, Q, F, T, newXdot, rb.getrbi());

    return newXdot;
}

StateVector Euler(const StateVector &X, const StateVector &Xdot, double dt){

  return X + dt * Xdot;
}

StateVector RK4(const StateVector &X, const StateVector &Xdot, double t, double dt, const int nbodies, const RBody &rb, const Environment &env) {
    StateVector K1(nbodies * STATE_SIZE), K2(nbodies * STATE_SIZE);
    StateVector K3(nbodies * STATE_SIZE), K4(nbodies * STATE_SIZE);

    K1 = dt * Xdot;
    K2 = dt * dynamics(X + 0.5 * K1, t + 0.5 * dt, dt, nbodies, rb, env);
    K3 = dt * dynamics(X + 0.5 * K2, t + 0.5 * dt, dt,  nbodies, rb, env);
    K4 = dt * dynamics(X + K3, t + dt, dt, nbodies, rb, env);

    //cout << "----------X: " << endl; X.print(); cout << endl;
    //cout << "----------K1: " << endl; K1.print(); cout << endl;
    //cout << "----------K2: " << endl; K2.print(); cout << endl;
    //cout << "----------K3: " << endl; K3.print(); cout << endl;
    //cout << "----------K4: " << endl; K4.print(); cout << endl;
    //cout << "----------((K1 + (2 * K2) + (2 * K3) + K4) / 6.0): " << endl; ((K1 + (2 * K2) + (2 * K3) + K4) / 6.0).print(); cout << endl;
    //cout << "----------(X + ((K1 + (2 * K2) + (2 * K3) + K4) / 6.0)): " << endl; (X + ((K1 + (2 * K2) + (2 * K3) + K4) / 6.0)).print(); cout << endl;

    return X + ((K1 + (2 * K2) + (2 * K3) + K4) / 6.0);
}

void RBSystem::takeTimestep(double t, double dt) {
    StateVector Xnew;
    Quaternion q;
    Vector3d x, p, l;

    // compute the rate of change of state
    for(int i = 0; i < nbodies; i ++) {
        if (rblist[i].getType() != 1) {
            StatetoX(rblist[i].getX(), rblist[i].getQ(),
                     rblist[i].getP(), rblist[i].getL(), Y, i);
            //cout << "Y: \n" << endl;
            //Y.print();
            Ydot = dynamics(Y, t, dt, nbodies, rblist[i], Env);

            //cout << "Ydot AFTER FIRST DYNAMICS() line 282: " << endl;
            //Ydot.print();

            Xnew = RK4(Y, Ydot, t, dt, nbodies, rblist[i], Env);

            //cout << "Xnew AFTER RK4() line 287: " << endl;
            //Xnew.print();
            //cout << endl;

            XtoState(x, q, p, l, Xnew, i);

            rblist[i].setICs(x, q, p, l);
            //cout << endl << endl << "rblist[" << i << "].print(): " << endl;
            //rblist[i].print();
            //cout << endl;
        }
    }

    //printsys();
}

void RBSystem::drawSys(){
    // draw strut/spring
    for(int i = 0; i < nbodies; i++)
        rblist[i].drawbody();
}

void RBSystem::printsys() {
    cout << endl << "-----------------------------------------" << endl;
    cout << "# OF RIGID BODIES: " << nbodies << endl << "     ";
    for (int i = 0; i < nbodies; i++)
        rblist[i].print();
    cout << endl << "Y: " << endl << "     ";
        Y.print();
    cout << endl << "Ydot: " << endl << "     ";
        Ydot.print();
    cout << endl << "Environment: " << endl << "     ";
        cout << "G: " << Env.G << endl << "     ";
        cout << "W: " << Env.W << endl << "     ";
        cout << "Viscosity: " << Env.Vis << endl;

}
