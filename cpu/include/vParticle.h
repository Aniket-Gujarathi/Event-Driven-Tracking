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

    void configure(int height, int width, double maxa, double maxb, int nBins)
    {
        rows = (height + maxb) * 2 + 1;
        cols = (width + maxa) * 2 + 1;
        offsety = rows/2;
        offsetx = cols/2;

        ds.resize(rows, cols);
        bs.resize(rows, cols);
        for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {

                int dy = i - offsety;
                int dx = j - offsetx;

                ds(i, j) = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
                bs(i, j) = (nBins-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);

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
    //int minr, maxr;
    int mina, maxa;
    int minb, maxb;

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
    //double r;
    double a;
    double b;

    double weight;

public:

    double score;


    vParticle();
    vParticle& operator=(const vParticle &rhs);

    //initialise etc.
    void initialiseParameters(int id, double minLikelihood, double negativeBias, double inlierParam, double variance, int angbuckets);
    void updateMinLikelihood(double value);
    void attachPCB(preComputedBins *pcb) { this->pcb = pcb; }

    void initialiseState(double x, double y, double a, double b);
    void randomise(int x, int y, int a, int b);

    void resetWeight(double value);
    void resetRadius(double value);
    void resetArea();
    void setContraints(int minx, int maxx, int miny, int maxy, int mina, int maxa, int minb, int maxb);
    void checkConstraints();
    void setNegativeBias(double value);
    void setInlierParameter(double value);


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
        double theta = atan2(dy, dx);
        //double sqrd = pcb->queryDistance((int)dy, (int)dx) - r;
        //double sqrd = sqrt(pow(dx, 2.0) + pow(dy, 2.0)) - r;
        double sqrd = sqrt(pow(dx*cos(theta), 2.0) + pow(dy*sin(theta), 2.0));

        double fsqrd = std::fabs(sqrd);

        //int a = 0.5 + (angbuckets-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);
        int al = pcb->queryBinNumber((int)dy, (int)dx);

        //OPTION 2

        if(sqrd > 1.0 + inlierParameter)
            return;

        double cval = 0;
        if(fsqrd < 1.0)
            cval = 1.0;
        else if(fsqrd < 1.0 + inlierParameter)
            cval = (1.0 + inlierParameter - fsqrd) / inlierParameter;
        if(cval) {
            double improve = cval - angdist[al];
            if(improve > 0) {
                angdist[al] = cval;
                score += improve;
                if(score >= likelihood) {
                    likelihood = score;
                    nw = n;
                }
            }
        } else {
            score -= negativeScaler;
        }

        return;


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
    inline double geta()  { return a; }
    inline double getb()  { return b; }
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
    int seedx, seedy, seeda, seedb;
    double nRandoms;

    //data
    std::vector<vParticle> ps;
    std::vector<vParticle> ps_snap;
    std::vector<double> accum_dist;
    preComputedBins pcb;
    std::vector<vPartObsThread *> computeThreads;

    //variables
    double pwsumsq;
    int abound_min;
    int abound_max;
    int bbound_min;
    int bbound_max;

public:

    double maxlikelihood;

    vParticlefilter() {}

    void initialise(int width, int height, int nparticles,
                    int bins, bool adaptive, int nthreads, double minlikelihood,
                    double inlierThresh, double randoms, double negativeBias);

    void setSeed(int x, int y, int a = 0, int b = 0);
    void resetToSeed();
    void setMinLikelihood(double value);
    void setInlierParameter(double value);
    void setNegativeBias(double value);
    void setAdaptive(bool value = true);

    void performObservation(const deque<AE> &q);
    void extractTargetPosition(double &x, double &y, double &a, double &b);
    void extractTargetWindow(double &tw);
    void performResample();
    void performPrediction(double sigma);

    std::vector<vParticle> getps();

};



#endif
