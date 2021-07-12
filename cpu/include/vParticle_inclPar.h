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

    void configure(int height, int width, int nBins, double maxrad=0)
    {
        rows = (height + maxrad) * 2 + 1;
        cols = (width + maxrad) * 2 + 1;
        
        offsety = rows/2;
        offsetx = cols/2;

        ds.resize(rows, cols);
        bs.resize(rows, cols);
        for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {

                // int k = 5;
                int dy = i - offsety;
                int dx = j - offsetx;
                
                // int dy1 = i - k;
                // int dy2 = i + k;
                    
                ds(i, j) = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
                bs(i, j) = (nBins-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);
                // bs(i, j) = (dy2 - dy1);

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

    void initialiseCircle(int r=0);

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
    // int minr, maxr;
    int mintheta, maxtheta;
    int minc, maxc;

    //temporary parameters (on update cycle)
    double likelihood;
    int nw;
    double predlike;
    int    outlierCount;
    int    inlierCount;
    yarp::sig::Vector angdist;
    yarp::sig::Vector negdist;

    //state and weight
    double x;
    double y;
    // double r;
    double theta;
    double c;

    double weight;

    double x_par, y_par;

public:

    double score;


    vParticle();
    vParticle& operator=(const vParticle &rhs);

    //initialise etc.
    void initialiseParameters(int id, double minLikelihood, double negativeBias, double inlierParam, double variance, int angbuckets);
    void updateMinLikelihood(double value);
    void attachPCB(preComputedBins *pcb) { this->pcb = pcb; }

    void initialiseState(double x, double y, double theta, double c);
    void randomise(int x, int y, int theta, int c);

    void resetWeight(double value);
    // void resetRadius(double value);
    void resetArea();
    void setContraints(int minx, int maxx, int miny, int maxy, int mintheta, int maxtheta, int minc, int maxc);
    void checkConstraints();
    void setNegativeBias(double value);
    void setInlierParameter(double value);


    double findIntersection(int &vx, int &vy, double &x, double &y, double &m, double &c);
    double findRoots(double a, double b, double c, int check);

    //update
    void predict(double sigma);
    double approxatan2(double y, double x);

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
        double dx = vx - x;
        double dy = vy - y;

        // // distance between the parabola and directrix (2 * a)
        double m = tan(theta*(M_PI / 180));
        double dist_par_dir = std::fabs((m * x - y + c) / (sqrt(1 + pow(m, 2)))) / 2.0;

        // Find pt of intersection of parabola and foc-ev
        // if (vy > y){
        //     return;
        // }
        // x_par, y_par = vParticle::findIntersection(vx, vy, x, y, m, c);

        // if (x_par== NULL || y_par == NULL || x_par > x + 65 || x_par < x - 65){
        //     return;
        // }

        // double dist_foc_par = std::fabs(sqrt(pow((x - x_par), 2) + pow((y - y_par), 2)));

        // double fdist_par = ((sqrt(pow((vx - x), 2) + pow((vy - y), 2))) - dist_foc_par);

        // // distance from focus
        double dist_focus = sqrt(pow(dx, 2) + pow(dy, 2));
        double fdist_focus = std::fabs(dist_focus);
        
        // // distance from directrix
        double dist_directrix = (m * vx - vy + c) / (sqrt(1 + pow(m, 2)));
        double fdist_directrix = std::fabs(dist_directrix);

        // int a = pcb->queryBinNumber((int)dy, (int)dx);
        
        // if(fdist_par > 60.0 || fdist_par < -60.0 || vy < m*vx + c){
        //     return;}
        
        // double cval = NULL;
        // if(fdist_par >= -5.0 && fdist_par <= 5.0)
        //     cval = 1.0;
        // else if (20.0 > fdist_par && fdist_par > 5.0){
        //     cval = -((fdist_par - 2) / 18);
        // }

        ////// parabola conditions for focus-directrix //////
        double dist_diff = std::fabs(fdist_directrix - fdist_focus);
        double cval = NULL;
        if(fdist_focus > 60.0 || dist_par_dir < 15.0 || vy < m*vx + c || vy > y + 10)
            return;
        else if(dist_diff <= 10.0)
            cval = 1.0;
        else if (fdist_directrix < 5.0)
            cval = -1.0;
        else if(fdist_directrix > fdist_focus)
            cval = 0.1;
        else if(fdist_directrix < fdist_focus)
            cval = -0.1;
        
        // double dist_diff = (fdist_directrix - fdist_focus);
        // double cval = NULL;
        // if(fdist_focus > 20.0 || dist_par_dir < 20.0 || vy < m*vx + c)
        //     return;
        // else if(dist_diff <= 5.0 && dist_diff >= -5.0)
        //     cval = 1.0;
        // else if (dist_diff < -5.0 && dist_diff > -20.0)
        //     cval = -0.5;
        // else if(dist_diff > 2.0 && dist_diff < 3.0)
        //     cval = 0.5;

        if(cval){
            if(cval >= 0.0) {
                double improve = cval;
                if(improve > 0) {
                    // angdist[a] = cval;
                    score += improve;
                    if(score >= likelihood) {
                        likelihood = score;
                        nw = n;
                    }
                }    
            } 
            else if (cval < 0.0) {
                score += cval;
            }
        }
        else{
            return;
        }
        
        
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
    // inline double getr()  { return r; }
    inline double gettheta()  { return theta; }
    inline double getc()  { return c; }
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
    int seedx, seedy, seedtheta, seedc;
    double nRandoms;

    //data
    std::vector<vParticle> ps;
    std::vector<vParticle> ps_snap;
    std::vector<double> accum_dist;
    preComputedBins pcb;
    std::vector<vPartObsThread *> computeThreads;

    //variables
    double pwsumsq;
    // int rbound_min;
    // int rbound_max;
    int theta_min;
    int theta_max;
    int c_min;
    int c_max;

public:

    double maxlikelihood;

    vParticlefilter() {}

    void initialise(int width, int height, int nparticles,
                    int bins, bool adaptive, int nthreads, double minlikelihood,
                    double inlierThresh, double randoms, double negativeBias);

    void setSeed(int x, int y, int theta = 0, int c = 0);
    void resetToSeed();
    void setMinLikelihood(double value);
    void setInlierParameter(double value);
    void setNegativeBias(double value);
    void setAdaptive(bool value = true);

    void performObservation(const deque<AE> &q);
    void extractTargetPosition(double &x, double &y, double &theta, double &c);
    void extractTargetWindow(double &tw);
    void performResample();
    void performPrediction(double sigma);

    std::vector<vParticle> getps();

};



#endif
