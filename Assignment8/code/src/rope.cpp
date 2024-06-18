/*
 * @Description: Description of this file;
 * @Version: 2.0
 * @Author: xm
 * @Date: 2020-04-24 13:18:48
 * @LastEditors: xm
 * @LastEditTime: 2024-06-18 20:41:01
 */
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
        Vector2D step = (end - start) / (num_nodes - 1);
        for (int i = 0; i < num_nodes; i ++ )
        {
            Mass* mass = new Mass(start + step * i, node_mass, false);
            mass->velocity = Vector2D(0,0);
            if(i > 0)
                springs.push_back(new Spring(masses.back(), mass, k));
            masses.push_back(mass);
        }
//        Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D ab = (s->m2)->position - (s->m1)->position;
            Vector2D f = -s->k * ab.unit() * (ab.norm() - s->rest_length);
            s->m1->forces -= f;
            s->m2->forces += f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                // TODO (Part 2): Add global damping
                float k_d = 0.01f;
                Vector2D f_d = -k_d * m->velocity;
                m->forces += f_d;
                // calculate position (semi-implicit Euler)
                Vector2D a = m->forces / m->mass;
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
            Vector2D ab = (s->m2)->position - (s->m1)->position;
            Vector2D f = -s->k * ab.unit() * (ab.norm() - s->rest_length);
            s->m1->forces -= f;
            s->m2->forces += f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                m->forces += gravity * m->mass;
                Vector2D a = m->forces / m->mass;
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D curPosition = m->position;
                Vector2D prePosition = m->last_position;
                Vector2D delta_x = curPosition - prePosition;
                m->last_position = curPosition;
                // TODO (Part 4): Add global Verlet damping
                float damping_factor = 0.00005f;
                m->position += (1 - damping_factor) * (delta_x + a * delta_t * delta_t);
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
