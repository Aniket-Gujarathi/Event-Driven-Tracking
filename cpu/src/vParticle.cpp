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

#include "vParticle.h"
#include <cmath>
#include <limits>
#include <algorithm>

using ev::event;
using ev::AddressEvent;

double generateGaussianNoise(double mu, double sigma)
{
    const double epsilon = std::numeric_limits<double>::min();
    const double two_pi = 2.0*3.14159265358979323846;

    static double z0, z1;
    static bool generate;
    generate = !generate;

    if (!generate)
       return z1 * sigma + mu;

    double u1, u2;
    do
     {
       u1 = rand() * (1.0 / RAND_MAX);
       u2 = rand() * (1.0 / RAND_MAX);
     }
    while ( u1 <= epsilon );

    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
}

double generateUniformNoise(double mu, double sigma)
{
    return mu + rand() * (2.0 * sigma / RAND_MAX) - sigma;
}

void drawEvents(yarp::sig::ImageOf< yarp::sig::PixelBgr> &image, deque<AE> &q,
                int offsetx) {

    if(q.empty()) return;

    //draw oldest first
    for(int i = (int)q.size()-1; i >= 0; i--) {
        double p = (double)i / (double)q.size();
        //auto v = is_event<AE>(q[i]);
        image(q[i].x + offsetx, q[i].y) =
                yarp::sig::PixelBgr(255 * (1-p), 0, 255);
    }
}

void drawcircle(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int cx, int cy,
                int cr, int id)
{

    for(int y = -cr; y <= cr; y++) {
        for(int x = -cr; x <= cr; x++) {
            if(fabs(sqrt(pow(x, 2.0) + pow(y, 2.0)) - (double)cr) > 0.8)
                continue;
            int px = cx + x; int py = cy + y;
            if(py<0 || py>(int)image.height()-1 || px<0 || px>(int)image.width()-1)
                continue;
            switch(id) {
            case(0): //green
                image(px, py) = yarp::sig::PixelBgr(0, 255, 0);
                break;
            case(1): //blue
                image(px, py) = yarp::sig::PixelBgr(0, 0, 255);
                break;
            case(2): //red
                image(px, py) = yarp::sig::PixelBgr(255, 0, 0);
                break;
            default:
                image(px, py) = yarp::sig::PixelBgr(255, 255, 0);
                break;

            }

        }
    }

}

void drawDistribution(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, std::vector<templatedParticle> &indexedlist)
{

    double sum = 0;
    std::vector<double> weights;
    for(unsigned int i = 0; i < indexedlist.size(); i++) {
        weights.push_back(indexedlist[i].weight);
        sum += weights.back();
    }

    std::sort(weights.begin(), weights.end());


    image.resize(indexedlist.size(), 100);
    image.zero();
    for(unsigned int i = 0; i < weights.size(); i++) {
        image(weights.size() - 1 -  i, 99 - weights[i]*100) = yarp::sig::PixelBgr(255, 255, 255);
    }
}

/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLEFILTER
/*////////////////////////////////////////////////////////////////////////////*/


void vParticlefilter::initialise(int width, int height, int nparticles,
                                 bool adaptive, int nthreads)
{
    res.width = width;
    res.height = height;
    this->nparticles = nparticles;
    this->adaptive = adaptive;
    this->nthreads = nthreads;

    setSeed(res.width/2.0, res.height/2.0);

    ps.clear();
    ps_snap.clear();
    accum_dist.resize(this->nparticles);

    if(this->nthreads > 1) {
        for(int i = 0; i < this->nthreads; i++) {
            int pStart = i * (this->nparticles / this->nthreads);
            int pEnd = (i+1) * (this->nparticles / this->nthreads);
            if(i == this->nthreads - 1)
                pEnd = this->nparticles;

            yInfo() << "Thread" << i << "=" << pStart << "->" << pEnd-1;
            computeThreads.push_back(new vPartObsThread(pStart, pEnd));
            computeThreads[i]->start();
        }
    }


    templatedParticle p;
    p.weight = 1.0 / (double)nparticles;

    vector<double> mins = {0.0, 0.0, 0.2};
    vector<double> maxs = {(double)res.width, (double)res.height, 0.7};
    p.setConstraints(mins, maxs);

    double max_likelihood = initialiseAsCircle(15);
    p.setAppearance(&appearance, max_likelihood);

    for(int i = 0; i < this->nparticles; i++) {
        p.id = i;
        ps.push_back(p);
        ps_snap.push_back(p);
    }

    resetToSeed();
}

void vParticlefilter::setAppearance(const ImageOf<PixelFloat> &new_appearance,
                                    double max_likelihood)
{
    this->appearance.copy(new_appearance);
    yInfo() << &new_appearance << " " << &(this->appearance);
    for(auto &p : ps) {
        p.setAppearance(&(this->appearance), max_likelihood);
        yInfo() << p.appearance;
    }


}

void vParticlefilter::setSeed(int x, int y, int r)
{
    seedx = x; seedy = y; seedr = r;
}

void vParticlefilter::resetToSeed()
{
    if(seedr) {
        for(int i = 0; i < nparticles; i++) {
            ps[i].state = {(double)seedx, (double)seedy, (double)seedr};
        }
    } else {
        for(int i = 0; i < nparticles; i++) {
            ps[i].state = {(double)seedx, (double)seedy, 1.0};
        }
    }
}

void vParticlefilter::setMinLikelihood(double value)
{
    yWarning() << "Deprecated";
    //for(int i = 0; i < nparticles; i++)
        //ps[i].updateMinLikelihood(value);
}

void vParticlefilter::setInlierParameter(double value)
{
    yWarning() << "Deprecated";
 //   for(int i = 0; i < nparticles; i++)
 //       ps[i].setInlierParameter(value);
}

void vParticlefilter::setNegativeBias(double value)
{
    yWarning() << "Deprecated";
  //  for(int i = 0; i < nparticles; i++)
   //     ps[i].setNegativeBias(value);
}

void vParticlefilter::setAdaptive(bool value)
{
    adaptive = value;
}

void vParticlefilter::performObservation(const deque<AE> &q)
{
    double normval = 0.0;
    if(nthreads == 1) {

        //START WITHOUT THREAD
        for(int i = 0; i < nparticles; i++) {
            ps[i].initLikelihood(q.size());
        }

        for(int i = 0; i < nparticles; i++) {
            for(int j = 0; j < (int)q.size(); j++) {
                ps[i].incrementalLikelihood(q[j].x, q[j].y, j);
            }
        }

        for(int i = 0; i < nparticles; i++) {
            ps[i].concludeLikelihood();
            normval += ps[i].weight;
        }

    } else {

        //START MULTI-THREAD
        for(int k = 0; k < nthreads; k++) {
            computeThreads[k]->setDataSources(&ps, &q);
            computeThreads[k]->process();
        }

        for(int k = 0; k < nthreads; k++) {
            normval += computeThreads[k]->waittilldone();
        }
    }

    normval = 1.0 / normval;

    pwsumsq = 0;
    maxlikelihood = 0;
    for(int i = 0; i < nparticles; i ++) {
        ps[i].normaliseWithinPopulation(normval);
        pwsumsq += pow(ps[i].weight, 2.0);
        maxlikelihood = std::max(maxlikelihood, ps[i].getl());
    }

}

void vParticlefilter::extractTargetPosition(double &x, double &y, double &r)
{
    x = 0; y = 0; r = 0;

    for(auto &p : ps) {
        x += p.state[templatedParticle::x] * p.weight;
        y += p.state[templatedParticle::y] * p.weight;
        r += p.getAbsoluteSize() * p.weight;
    }

//    for(int i = 0; i < nparticles; i++) {
//        x += ps[i].state[templatedParticle::x] * ps[i].weight;
//        y += ps[i].state[templatedParticle::y] * ps[i].weight;
//        //r += ps[i].state[templatedParticle::s] * ps[i].weight;
//        r += ps[i].getAbsoluteSize() * ps[i].weight;
//    }
}

void vParticlefilter::extractTargetWindow(double &tw)
{
    tw = 0;
    for(auto &p : ps)
        tw += p.getn() * p.weight;

//    for(int i = 0; i < nparticles; i++) {
//        double w = ps[i].weight;
//        tw += ps[i].getn() * w;
//
//    }

}

void vParticlefilter::performResample()
{
    if(!adaptive || pwsumsq * nparticles > 2.0) {
        //initialise for the resample
        double accum = 0;
        for(int i = 0; i < nparticles; i++) {
            ps_snap[i] = ps[i];
            accum += ps[i].weight;
            accum_dist[i] = accum;
        }

        //perform the resample
        for(int i = 0; i < nparticles; i++) {
            double rn = (double)rand() / RAND_MAX;
                int j = 0;
                for(j = 0; j < nparticles; j++)
                    if(accum_dist[j] > rn) break;
                ps[i] = ps_snap[j];
        }
    }
}


void vParticlefilter::performPrediction(double sigma)
{
    for(int i = 0; i < nparticles; i++)
        ps[i].predict(sigma);
}

std::vector<templatedParticle> vParticlefilter::getps()
{
    return ps;
}

double vParticlefilter::initialiseAsCircle(int r)
{
    appearance.resize(r*2+1, r*2+1);
    appearance.zero();

    double max_likelihood = 0;

    for(size_t y = 0; y < appearance.height(); y++) {
        for(size_t x = 0; x < appearance.width(); x++) {

            double d = sqrt((y - r) * (y - r) + (x - r) * (x - r)) - r;

            if(d > 2.0)
                appearance(y, x) = 0.0;

            else if(d < -2.0)
                appearance(y, x) = -1.0;

            else if(d < 1.0 && d > -1.0)
            {
                appearance(y, x) = 1.0;
                max_likelihood++;
            }
            else
            {
                appearance(y, x) = (2.0 - fabs(d));
                max_likelihood += appearance(y, x);
            }
        }
    }

    for(size_t y = 0; y < appearance.height(); y++) {
        for(size_t x = 0; x < appearance.width(); x++) {
            std::cout << appearance(y, x) << " ";
        }
        std::cout << std::endl;
    }

    return max_likelihood;

}

/*////////////////////////////////////////////////////////////////////////////*/
//particleobserver (threaded observer)
/*////////////////////////////////////////////////////////////////////////////*/

vPartObsThread::vPartObsThread(int pStart, int pEnd)
{
    this->pStart = pStart;
    this->pEnd = pEnd;
    processing.lock();
    done.lock();
}

void vPartObsThread::setDataSources(std::vector<templatedParticle> *particles,
                                    const deque<AE> *stw)
{
    this->particles = particles;
    this->stw = stw;
}

void vPartObsThread::process()
{
    processing.unlock();
}

double vPartObsThread::waittilldone()
{
    done.lock();
    return normval;
}

void vPartObsThread::run()
{
    while(!isStopping()) {

        processing.lock();
        if(isStopping()) return;

        int nw = (*stw).size();

        for(int i = pStart; i < pEnd; i++) {
            (*particles)[i].initLikelihood(nw);
        }

        for(int i = pStart; i < pEnd; i++) {
            for(int j = 0; j < nw; j++) {
                //AE* v = read_as<AE>((*stw)[j]);
                (*particles)[i].incrementalLikelihood((*stw)[j].x, (*stw)[j].y, j);
            }
        }

        normval = 0.0;
        for(int i = pStart; i < pEnd; i++) {
            (*particles)[i].concludeLikelihood();
            normval += (*particles)[i].weight;
        }

        done.unlock();

    }

}


/*////////////////////////////////////////////////////////////////////////////*/
// templatedParticle
/*////////////////////////////////////////////////////////////////////////////*/

templatedParticle::templatedParticle()
{
    appearance = nullptr;
}

templatedParticle& templatedParticle::operator=(const templatedParticle &rhs)
{
    this->state = rhs.state;
    this->weight = rhs.weight;
    return *this;
}

void templatedParticle::setAppearance(ImageOf<PixelFloat> *appearance, double max_likelihood)
{
    this->appearance = appearance;
    this->max_likelihood = max_likelihood;
    this->min_likelihood = 0.03 * max_likelihood;
    this->offset_x = appearance->width() / 2;
    this->offset_y = appearance->height() / 2;
}

bool templatedParticle::setConstraints(vector<double> mins, vector<double> maxs)
{
    //if(mins.size() != state.size()) return false;
    //if(maxs.size() != state.size()) return false;

    min_state = mins;
    max_state = maxs;
    constrain = true;

    return true;
}

void templatedParticle::predict(double sigma)
{
    state[x] = generateUniformNoise(state[x], sigma);
    state[y] = generateUniformNoise(state[y], sigma);
    state[s] = generateUniformNoise(state[s], 0.05);

    if(constrain) checkConstraints();

}

void templatedParticle::checkConstraints()
{

    for(size_t i = 0; i < state.size(); i++) {
        if(state[i] < min_state[i]) state[i] = min_state[i];
        if(state[i] > max_state[i]) state[i] = max_state[i];
    }

}
