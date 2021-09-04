#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
        Vector2D interval = (end-start)/(num_nodes-1);
        for (int i = 0; i<num_nodes;++i){
            Mass* mass = new Mass(start+i*interval, node_mass, false);
            masses.push_back(mass);
        }
        for (int i = 1; i<num_nodes;++i){
            Spring* spring  = new Spring(masses[i-1],masses[i],k);
            springs.push_back(spring);
        }
        for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D ba = s->m2->position - s->m1->position;
            Vector2D f_ba = - s->k * ba.unit() * (ba.norm() - s->rest_length);
            s->m2->forces+=f_ba;
            s->m1->forces-=f_ba;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {   // TODO (Part 2): Add global damping
                double k_d = 0.01;
                m->forces += -k_d * m->velocity;
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += m->mass * gravity;
                Vector2D a = m->forces / m->mass;

                //For explicit method
                // m->position += m->velocity * delta_t;
                // m->velocity += a * delta_t;

                //For semi-implicit method
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;
                
            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D ba = s->m2->position - s->m1->position;
            Vector2D delta_ba = - ba.unit() * (ba.norm() - s->rest_length) * 0.5;
            s->m2->position+=s->m2->pinned?Vector2D(0,0):(s->m1->pinned?delta_ba*2:delta_ba);
            s->m1->position-=s->m1->pinned?Vector2D(0,0):(s->m2->pinned?delta_ba*2:delta_ba);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {   
                m->forces += m->mass * gravity;
                Vector2D a = m->forces / m->mass;

                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                // TODO (Part 4): Add global Verlet damping
                
                double damping_factor = 0.00005;
                m->position = m->position + (1-damping_factor)*(m->position-m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
