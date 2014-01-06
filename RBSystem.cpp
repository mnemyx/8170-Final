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
    trylist = new RBody[nbodies];

    for(int i = 0; i < nbodies; i++) {
        rblist[i].setRBI(i);
        trylist[i].setRBI(i);
    }

    Y.setSize(nbodies * STATE_SIZE);
    Ydot.setSize(nbodies * STATE_SIZE);

    xextents.setMaxBodies(nbodies);
    xextents.setListType(XEXT);

    yextents.setMaxBodies(nbodies);
    yextents.setListType(YEXT);

    zextents.setMaxBodies(nbodies);
    zextents.setListType(ZEXT);

    alloverlaps.setSize(nbodies);
    allcollisions.setSize(nbodies);

    allcontacts.setSize(nbodies);
}

RBSystem::~RBSystem() {
    delete []rblist;
    delete []trylist;
}

void RBSystem::setParams(double m[], double width[], double height[], double depth[], int type[], double d1[], double d2[], double d3[], Vector4d c[]) {
    //cout << "nbodies -- setParams(): " << nbodies << endl;
    for(int i = 0; i < nbodies; i++) {
        //cout << "width[" << i << "]: " << width[i] << endl;
        //cout << "height[" << i << "]: " << height[i] << endl;
        //cout << "depth[" << i << "]: " << depth[i] << endl;
        rblist[i].setParams(m[i], width[i], height[i], depth[i], type[i], d1[i], d2[i], d3[i]);
        rblist[i].setColor(c[i]);

        trylist[i].setParams(m[i], width[i], height[i], depth[i], type[i], d1[i], d2[i], d3[i]);
        trylist[i].setColor(c[i]);
    }
}

void RBSystem::setEnv(Vector3d g, Vector3d w, double v) {
    Env.G = g;
    Env.W = w;
    Env.Vis = v;
}

void RBSystem::initializeState(Vector3d x0[], Quaternion q[], Vector3d v0[], Vector3d omega0[]) {
    xextents.Clear();
    yextents.Clear();
    zextents.Clear();

    for(int i = 0; i < nbodies; i++) {
        rblist[i].initICs(x0[i], q[i], v0[i], omega0[i]);
        //rblist[i].ComputeAuxiliaries();

        trylist[i].initICs(x0[i], q[i], v0[i], omega0[i]);
        //trylist[i].ComputeAuxiliaries();
        //rblist[i].print();

        xextents.insertBody(&rblist[i]);
        yextents.insertBody(&rblist[i]);
        zextents.insertBody(&rblist[i]);
    }

    xextents.Sort(1);
    //xextents.print();

    yextents.Sort(1);
    //yextents.print();

    zextents.Sort(1);
    //zextents.print();

    alloverlaps.Clear();
    alloverlaps.MergeOverlaps(xextents.Overlaps(), yextents.Overlaps(), zextents.Overlaps());
    alloverlaps.FindWitnesses();
    //alloverlaps.print();

    allcontacts.Clear();
    allcontacts.ExtractContacts(alloverlaps);
    //allcontacts.print();
    cout << endl;

}

void XtoState(Vector3d &x, Quaternion &q, Vector3d &p, Vector3d &l, const StateVector X, const int rbi) {
    int i = rbi * 13;
    //X.print();
    x.set(X[i],X[i+1],X[i+2]);
      //cout << "x: " << x << endl;
    q.q.w = X[i+3];
    q.q.x = X[i+4];
    q.q.y = X[i+5];
    q.q.z = X[i+6];
      //cout << "q: " << q << endl;
    p.set(X[i+7],X[i+8],X[i+9]);
      //cout << "p: " << p << endl;
    l.set(X[i+10],X[i+11], X[i+12]);
      //cout << "l: " << l << endl;
}

void StatetoX(const Vector3d x, const Quaternion q, const Vector3d p, const Vector3d l, StateVector &X, const int rbi) {
    int i = rbi * 13;

    X[i] = x.x;
    X[i+1] = x.y;
    X[i+2] = x.z;
    X[i+3] = q.q.w;
    X[i+4] = q.q.x;
    X[i+5] = q.q.y;
    X[i+6] = q.q.z;
    X[i+7] = p.x;
    X[i+8] = p.y;
    X[i+9] = p.z;
    X[i+10] = l.x;
    X[i+11] = l.y;
    X[i+12] = l.z;

    //cout << X[i+12] << endl;

    //cout << "i+12: " << i + 12 <<endl;
}

StateVector dynamics(const StateVector &X, double t, double dt, const int nbodies, const RBody &rb, const Environment &Env) {
    Vector3d x, p, l;
    Quaternion q;
    StateVector newXdot(nbodies * STATE_SIZE);
    Vector3d F, T, V, Q;
    Vector3d fg;
    Vector3d w;
    Matrix3x3 r, iinv;
    Quaternion wq, qdot;

    //cout << "X prior to XtoState() in dynamics() line 180: " << endl;
    //X.print();
    //cout << endl;

    //cout << rb.getrbi() << endl;
    XtoState(x, q, p, l, X, rb.getrbi());

    //cout << "after XtoState" << endl;
    //cout << "x: " << x << endl;
    //cout << "q: " << q << endl;
    //cout << "p: " << p << endl;
    //cout << "l: " << l << endl;

    // calc velocity
    V = p * rb.getMinv();
    //cout << "M: " << rb.getM() << endl;
    //cout << "V: " << V << endl;

    // calc rate of change of q:
    r = q.normalize().rotation();
    //cout << "q: " << q << endl;
    //cout << "q.normalize(): " << q.normalize() << endl;
    //q.normalize().rotation().print();


    //r.print();
    iinv = r * rb.getIbodyinv() * r.transpose();


    //cout << "iinv: \n";
    //iinv.print();
    //cout << "l: " << l << endl;
    //cout << "iinv * l: " << iinv * l << endl;
    w = iinv * l;
    wq.set(w);
    //cout << endl << "w: " << w << endl;
    //cout << endl << "wq: " << wq << endl;


    Q = 0.5 * wq * q;
    //cout << "Q: " << Q << endl;
    // calc force
    //fg = rb.getM() * Env.G;

    if (Env.W.x == 0 && Env.W.y == 0 && Env.W.z == 0)
        fg = (Env.G - Env.Vis * V) * rb.getM();
    else
        fg = (Env.G + Env.Vis * (Env.W - V)) * rb.getM();

    F = fg;// + fs + fd;
    //if(t <= 5) {
        //F = F + fa;
    //}
    // calc torque
    //ts = (p1 - x) % fs;
    //td = (p1 - x) % fd;

    //T = ts + td;
    T.set(0,0,0);
    //if(t <= 5) {
        //T = T + ((rb.getvertex(0) - x) % fa);
    //}

    StatetoX(V, Q, F, T, newXdot, rb.getrbi());
    return newXdot;
}

StateVector Euler(const StateVector &X, const StateVector &Xdot, double dt) {
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
    //cout << "----------Xdot: " << endl; Xdot.print(); cout << endl;
    //cout << "----------K1: " << endl; K1.print(); cout << endl;
    //cout << "----------K2: " << endl; K2.print(); cout << endl;
    //cout << "----------K3: " << endl; K3.print(); cout << endl;
    //cout << "----------K4: " << endl; K4.print(); cout << endl;
    //cout << "----------((K1 + (2 * K2) + (2 * K3) + K4) / 6.0): " << endl; ((K1 + (2 * K2) + (2 * K3) + K4) / 6.0).print(); cout << endl;
    //cout << "----------(X + ((K1 + (2 * K2) + (2 * K3) + K4) / 6.0)): " << endl; (X + ((K1 + (2 * K2) + (2 * K3) + K4) / 6.0)).print(); cout << endl;

    return X + ((K1 + (2 * K2) + (2 * K3) + K4) / 6.0);
}


void RBSystem::takeFullStep(double t, double dt) {
    StateVector newX;
    Quaternion q;
    Vector3d x, p, l;

    for(int i = 0; i < nbodies; i ++) {
        //if (rblist[i].getType() != 1) {
            newX = RK4(Y, Ydot, t, dt, nbodies, rblist[i], Env);
            XtoState(x, q, p, l, newX, i);

            cout << "x: " << x << endl;
            cout << "q: " << q << endl;
            cout << "p: " << p << endl;
            cout << "l: " << l << endl;

            rblist[i].setICs(x, q, p, l);
        //}
    }
}

void RBSystem::takeTimestep(double t, double dt) {
    StateVector rbSV;
    Quaternion q;
    Vector3d x, p, l;
    for(int i = 0; i < nbodies; i ++) {
        //if (rblist[i].getType() != 1) {
            StatetoX(rblist[i].getX(), rblist[i].getQ(), rblist[i].getP(), rblist[i].getL(), Y, i);
            //cout << "Y: " << endl;
            //Y.print();
            //cout << endl;
            rbSV = dynamics(Y, t, dt, nbodies, rblist[i], Env);

            XtoState(x, q, p, l, rbSV, i);
            StatetoX(x, q, p, l, Ydot, i);

            //cout << "i: " << i << endl;
            //Ydot.print(); cout << endl;
            //cout << "Ydot: " <<endl;
            //Ydot.print();
            //cout << endl;
        //}
    }
}

bool RBSystem::checkCollisions(double t, double dt) {
    StateVector xeuler;
    Quaternion q;
    Vector3d x, p, l;

    for(int i = 0; i < nbodies; i ++) {
        //if (rblist[i].getType() != 1) {
        //Y.print();
        //cout << "^^^^^^" << endl;
        //Ydot.print();
            xeuler = Euler(Y, Ydot, dt);
            XtoState(x, q, p, l, xeuler, i);

            //cout << "xeuler: " << endl;
            //xeuler.print();
            //cout << endl;
            rblist[i].setICs(x, q, p, l);
        //}
    }

    xextents.UpdateExtents();
    //xextents.print();
    yextents.UpdateExtents();
    //yextents.print();
    zextents.UpdateExtents();
    //zextents.print();

    alloverlaps.MergeOverlaps(xextents.Overlaps(), yextents.Overlaps(), zextents.Overlaps());
    alloverlaps.FindWitnesses();
    //alloverlaps.print();

    allcontacts.ExtractContacts(alloverlaps);
    allcontacts.print();
    //cout << endl;
    //printsys();

    if(allcontacts.ncontacts > 0)
        return true;
    else
        return false;
}

void RBSystem::handleCollisions(double t, double dt) {
    Quaternion dq, q;
    Vector3d dx, x, dp, p, dl, l;
    Vector3d ptang, pnorm;
    double dpt, pt, dxn, d, fc;
    StateVector Xc, Xnew;
    Vector3d fj;

    Contact *collided = allcontacts.First();

    while(collided != NULL) {
        // find the tangent and normal components to collision surface...
        pnorm = (collided->n * collided->p) * collided->p;
        cout << "pnorm: " << pnorm << endl;
        //pnorm = collided->n.normalize();
        ptang = collided->p - pnorm;

        //Y.print();
        // variables before collision...
        XtoState(x, q, p, l, Y, collided->a->rbi);

        // variables after collission, assuming that checkCollisions() really stored them into the rblist
        dx = collided->a->getX();
        dq = collided->a->getQ();
        dp = collided->a->getP();
        dl = collided->a->getL();

        //cout << "x: " << x << ", dx: " << dx << endl;
        //cout << "q: " << q << ", dq: " << dq << endl;
        //cout << "p: " << p << ", dp: " << dp << endl;
        //cout << "l: " << l << ", dl: " << dl << endl;

        dpt = dp * ptang;
        pt = p * ptang;
        dxn = dx * pnorm;
        d = (x - collided->p) * pnorm;

        //cout << "pt: " << pt << ", dpt: " << dpt << endl;
        cout << "d: " << d << ", dxn: " << dxn << endl;

        fc = -d / dxn;
        cout << "time ratio: " << fc << endl;

        /***** based on siggraph notes *******/
        Vector3d padot = collided->a->dpdt(collided->p);
        Vector3d pbdot = collided->b->dpdt(collided->p);
        Vector3d nt0 = collided->n;
        Vector3d ra = collided->p - collided->a->shape->GetCenter();
        Vector3d rb = collided->p - collided->a->shape->GetCenter();

        double vrel = nt0 * (padot - pbdot);
        double numerator = -(1 + .8) * vrel;

        double term1 = collided->a->getMinv();
        double term2 = collided->b->getMinv();
        double term3 = nt0 * ((collided->a->getIinv() * (ra % nt0)) % ra);
        double term4 = nt0 * ((collided->b->getIinv() * (rb % nt0)) % rb);

        double j = numerator / (term1 + term2 + term3 + term4);

        fj = j * nt0;

        /***** PREVIOUS BASED ON HOUSE'S ******/
        // calculate impulse and apply...
        //fj = collided->impulse() * collided->n;

        // get collision or stop; use euler
        Xc = Euler(Y, Ydot, fc * dt);       //critical point

        //if(collided->a->rbtype != 1) {
            XtoState(x, q, p, l, Xc, collided->a->rbi);
            collided->a->setICs(x, q, (p)+fj, (l)+(ra % fj));
            StatetoX(collided->a->getX(), collided->a->getQ(), collided->a->getP(), collided->a->getL(), Ydot, collided->a->rbi);
        //}

        //if(collided->b->rbtype != 1) {
            XtoState(x, q, p, l, Xc, collided->b->rbi);
            collided->b->setICs(x, q, (p)-fj, (l)-(rb % fj));
            StatetoX(collided->b->getX(), collided->b->getQ(), collided->b->getP(), collided->b->getL(), Ydot, collided->b->rbi);
        //} **/

        // finish off timestep...?
        Xnew = Euler(Xc, Ydot, (1 - fc) * dt);

        XtoState(x, q, p, l, Xnew, collided->a->rbi);
        collided->a->setICs(x, q, p, l);

        XtoState(x, q, p, l, Xnew, collided->b->rbi);
        collided->b->setICs(x, q, p, l);

        cout << "Xnew: "<<endl; Xnew.print();

        collided = allcontacts.Next();
    }

    allcontacts.Clear();
    // time to event: resting slide-to-stop time fraction
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
