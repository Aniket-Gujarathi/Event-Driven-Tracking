/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __VPARTICLE__
#define __VPARTICLE__

#include <event-driven/all.h>
#include <yarp/sig/all.h>

using namespace ev;

class vParticle;

void drawEvents(yarp::sig::ImageOf< yarp::sig::PixelBgr> &image, deque<AE> &q, int offsetx = 0);

void drawcircle(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int cx, int cy, int cr, int id = 0);

void drawDistribution(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, std::vector<vParticle> &indexedlist);

class preComputedBins;


class preComputedBins
{

private:

    yarp::sig::Matrix ds;
    yarp::sig::Matrix bs;
    int rows;
    int cols;
    int offsetx;
    int offsety;

public:

    preComputedBins()
    {
        rows = 0;
        cols = 0;
        offsetx = 0;
        offsety = 0;
    }

    void configure(int height, int width, double maxrad, int nBins)
    {
        rows = (height + maxrad) * 2 + 1;
        cols = (width + maxrad) * 2 + 1;
        
        offsety = rows/2;
        offsetx = cols/2;

        // std::vector<double> bins;
        // int k = 5;
        // bins.resize(2 * k);

        ds.resize(rows, cols);
        bs.resize(rows, cols);
        for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {

                int dy = i - offsety;
                int dx = j - offsetx;
                

                ds(i, j) = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
                bs(i, j) = (nBins-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);
                // bs(i, j) = (nBins-1) * bins.at(i);

            }
        }
    }

    inline double queryDistance(int dy, int dx)
    {
        dy += offsety; dx += offsetx;
//        if(dy < 0 || dy > rows || dx < 0 || dx > cols) {
//            std::cout << "preComputatedBins not large enough" << std::endl;
//            return 0.0;
//        }
        return ds(dy, dx);
    }

    inline int queryBinNumber(double dy, double dx)
    {
        dy += offsety; dx += offsetx;
//        if(dy < 0 || dy > rows || dx < 0 || dx > cols) {
//            std::cout << "preComputatedBins not large enough" << std::endl;
//            return 0.0;
//        }
        return (int)(bs(dy, dx) + 0.5);
    }

};

class projectedModel {

private:
    yarp::sig::ImageOf<yarp::sig::PixelFloat> projection;

public:

    projectedModel();

    void initialiseCircle(int r);

    inline double query(int x, int y);

};

/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLETRACKER
/*////////////////////////////////////////////////////////////////////////////*/
class vParticle
{
private:

    //static parameters
    int id;
    double minlikelihood;
    double inlierParameter;
    double variance;
    int angbuckets;
    preComputedBins *pcb;
    double negativeBias;
    double negativeScaler;

    bool constrain;
    int minx, maxx;
    int miny, maxy;
    int minr, maxr;
    int mintheta, maxtheta;
    int minc, maxc;
    int minxc, maxxc, minyc, maxyc;

    //temporary parameters (on update cycle)
    double likelihood;
    int nw;
    double predlike;
    int outlierCount;
    int inlierCount;
    yarp::sig::Vector angdist;
    yarp::sig::Vector negdist;

    //state and weight
    double x;
    double y;
    double theta;
    double c;
    //circle
    double xc;
    double yc;
    double r; //circle rad

    double weight;

public:

    double score;


    vParticle();
    vParticle& operator=(const vParticle &rhs);

    //initialise etc.
    void initialiseParameters(int id, double minLikelihood, double negativeBias, double inlierParam, double variance, int angbuckets);
    void updateMinLikelihood(double value);
    void attachPCB(preComputedBins *pcb) { this->pcb = pcb; }

    void initialiseState(double x, double y, double r, double theta, double c, double xc, double yc);
    void randomise(int x, int y, int r, int theta, int c, int xc, int yc);

    void resetWeight(double value);
    void resetRadius(double value);
    void resetArea();
    void setContraints(int minx, int maxx, int miny, int maxy, int minr, int maxr, int mintheta, int maxtheta, int minc, int maxc, int minxc, int maxxc, int minyc, int maxyc);
    void checkConstraints();
    void setNegativeBias(double value);
    void setInlierParameter(double value);


    //update
    void predict(double sigma);
    double approxatan2(double yc, double xc);

    void initLikelihood(int windowSize)
    {
        likelihood = minlikelihood;
        inlierCount = 0;
        outlierCount = 0;
        //angdist = 1.0 + inlierParameter;
        angdist = 0;
        score = 0;
        nw = windowSize;
        resetArea();
    }

    inline void incrementalLikelihood(int vx, int vy, int n)
    {
        // parabola
        double dx = vx - x;
        double dy = vy - y;
        // circle
        double dxc = vx - xc;
        double dyc = vy - yc;

        // distance between the circle centre and parabola focus
        // double dist_centres = std::fabs(pcb->queryDistance((yc - y), (xc - x)));
        double dist_centres = std::fabs(sqrt(pow((xc - x), 2) + pow((yc - y), 2)));
        
        // distance between the parabola and directrix (2 * a)
        double m = tan(theta*(M_PI / 180));
        double dist_par_dir = std::fabs((m * x - y + c) / (sqrt(1 + pow(m, 2)))) / 2.0;

        // distance from circle (boundary)
        // double dist_circ = pcb->queryDistance((int)dyc, (int)dxc) - r;
        double dist_circ = sqrt(dyc*dyc + dxc*dxc) - r;
        double fdist_circ = std::fabs(dist_circ);

        // distance from focus
        // double dist_focus = pcb->queryDistance((int)dy, (int)dx);
        double dist_focus = sqrt(dy*dy + dx*dx);
        double fdist_focus = std::fabs(dist_focus);
        
        // distance from directrix
        double dist_directrix = (m * vx - vy + c) / (sqrt(1 + pow(m, 2)));
        double fdist_directrix = std::fabs(dist_directrix);

        int a = pcb->queryBinNumber((int)dyc, (int)dxc);
   
        double cval = 0;
        
        if(fdist_focus > 10.0 || dist_par_dir < 5.0 || dist_centres > 5.0 || vy < m * vx + c)
            return;
        else if(fdist_directrix == fdist_focus || fdist_circ <= 1.0)
            cval = 1.0;
        else if (fdist_directrix < 2.0)
            cval = -0.5;
        else if(fdist_directrix > fdist_focus || fdist_circ < 1.0 + inlierParameter)
            cval = 0.5;
        
        if(cval) {
            // double improve = cval - angdist[a];
            double improve = cval;
            if(improve > 0) {
                // angdist[a] = cval;
                score += improve;
                if(score >= likelihood) {
                    // yDebug() << score << likelihood;
                    likelihood = score;
                    nw = n;
                }
            }
            
        } else {
            score -= negativeScaler;
        }
        
        return;


        // OPTION Parabola
        // double fsqrd_diff = std::fabs(fsqrd_dir - fsqrd_par);
        // double cval = 0;
        // if(fdist_dir > 1.0 + inlierParameter && fdist_par > 1.0 + inlierParameter)
        //     return;

        // else if (fdist_dir < fdist_par){
        //     score -= negativeScaler;
        // }
        // else if(fdist_dir == fdist_par){
        //     score += 1;
        //     yDebug() << "score_on"  << score;
        //     if(score >= likelihood) {
        //         likelihood = score;
        //         nw = n;
        //         }
            
        // }
        // else if(fdist_dir > fdist_par && fdist_dir < 1.0 + inlierParameter){
        //     score += 0.50;
        //     yDebug() << "score_in"  << score;
        //     if(score >= likelihood) {
        //         likelihood = score;
        //         nw = n;
        //         }
        // }
        
        //OPTION 2
        // double fsqrd = std::fabs(fdist_dir - fdist_par);

        

        //ORIGINAL
//        if(sqrd > inlierParameter) {
//            return;
//        }

//        if(sqrd > -inlierParameter) {
//            //int a = 0.5 + (angbuckets-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);

//            //int a = pcb->queryBinNumber((int)dy, (int)dx);

//            if(!angdist[a]) {
//                inlierCount++;
//                angdist[a] = 1;

//                score = inlierCount - (negativeScaler * outlierCount);
//                if(score >= likelihood) {
//                    likelihood = score;
//                    nw = n;
//                }

//            }

//        } else {
//            outlierCount++;
//        }

    }

    void concludeLikelihood()
    {

        //if(likelihood < minlikelihood) nw = n;
        weight = likelihood * weight;
    }

    void updateWeightSync(double normval);

    //get
    inline int    getid() { return id; }
    inline double getx()  { return x; }
    inline double gety()  { return y; }
    inline double getr()  { return r; }
    inline double gettheta()  { return theta; }
    inline double getc()  { return c; }
    inline double getxc()  { return xc; }
    inline double getyc()  { return yc; }
    inline double getw()  { return weight; }
    inline double getl()  { return likelihood; }
    inline double getnw() { return nw; }


};

/*////////////////////////////////////////////////////////////////////////////*/
// vParticleObserver
/*////////////////////////////////////////////////////////////////////////////*/
class vPartObsThread : public yarp::os::Thread
{
private:

    yarp::os::Mutex processing;
    yarp::os::Mutex done;
    int pStart;
    int pEnd;

    double normval;

    std::vector<vParticle> *particles;
    const deque<AE> *stw;

public:

    vPartObsThread(int pStart, int pEnd);
    void setDataSources(std::vector<vParticle> *particles, const deque<AE> *stw);
    void process();
    double waittilldone();

    void run();
};

/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLEFILTER
/*////////////////////////////////////////////////////////////////////////////*/

class vParticlefilter
{
private:

    //parameters
    int nparticles;
    int nthreads;
    ev::resolution res;
    bool adaptive;
    int bins;
    int seedx, seedy, seedr, seedtheta, seedc, seedxc, seedyc;
    double nRandoms;

    //data
    std::vector<vParticle> ps;
    std::vector<vParticle> ps_snap;
    std::vector<double> accum_dist;
    preComputedBins pcb;
    std::vector<vPartObsThread *> computeThreads;

    //variables
    double pwsumsq;
    int rbound_min;
    int rbound_max;
    int theta_min;
    int theta_max;
    int c_min;
    int c_max;
    int xc_min, xc_max, yc_min, yc_max;

public:

    double maxlikelihood;

    vParticlefilter() {}

    void initialise(int width, int height, int nparticles,
                    int bins, bool adaptive, int nthreads, double minlikelihood,
                    double inlierThresh, double randoms, double negativeBias);

    void setSeed(int x, int y, int r = 0, int theta = 0, int c = 0, int xc = 0, int yc = 0);
    void resetToSeed();
    void setMinLikelihood(double value);
    void setInlierParameter(double value);
    void setNegativeBias(double value);
    void setAdaptive(bool value = true);

    void performObservation(const deque<AE> &q);
    void extractTargetPosition(double &x, double &y, double &r, double &theta, double &c, double &xc, double &yc);
    void extractTargetWindow(double &tw);
    void performResample();
    void performPrediction(double sigma);

    std::vector<vParticle> getps();

};



#endif
