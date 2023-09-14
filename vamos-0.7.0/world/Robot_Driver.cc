//  Robot_Driver.cc - a computer-controlled driver
//
//	Vamos Automotive Simulator
//  Copyright (C) 2008 Sam Varner
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#include "Robot_Driver.h"
#include "../body/Car.h"
#include "../body/Wheel.h"
#include "../geometry/Calculations.h"
#include "../geometry/Constants.h"
#include "../geometry/Numeric.h"
#include "../geometry/Parameter.h"
#include "../geometry/Three_Vector.h"
#include "../track/Strip_Track.h"
#include "World.h"

#include <algorithm>
#include <limits>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Track;
using namespace Vamos_World;

//-----------------------------------------------------------------------------
Robot_Driver::Robot_Driver (Car* car_in, Strip_Track* track_in, double gravity) 
: Driver (car_in),
  mp_cars (0),
  m_speed_control (4.0, 0.0, 0.0),
  m_traction_control (0.5, 0.0, 0.0),
  m_brake_control (1.0, 0.0, 0.0),
  m_steer_control (0.5, 0.0, 0.0),
  m_target_slip (car_in->get_robot_parameters ().slip_ratio),
  m_road_index (0),
  m_segment_index (0),
  m_target_segment (0),
  mp_track (track_in),
  m_shift_time (0.0),
  m_short_shift (false),
  m_last_throttle (0.0),
  m_state (PARKED),
  m_state_time (0.0),
  m_time_step (1.0),
  m_lane_shift (0.0),
  m_lane_shift_timer (0.0),
  m_interact (true),
  m_show_steering_target (false),
  m_racing_line (mp_track->get_road (m_road_index),
                 car_in->get_robot_parameters ().lateral_acceleration,
                 gravity),
  m_braking (mp_track->get_road (m_road_index),
             car_in->get_robot_parameters ().deceleration,
             gravity,
             m_racing_line)
{
  m_traction_control.set (m_target_slip);
}

Robot_Driver::~Robot_Driver ()
{
}

// Step the driver forward in time.
void 
Robot_Driver::propagate (double time_step) 
{
  m_time_step = time_step;
  if (m_lane_shift != 0.0)
    m_lane_shift_timer += time_step;

  switch (m_state)
    {
    case PARKED:
      set_brake (1.0);
      mp_car->shift (0);
      mp_car->disengage_clutch (0.0);
      if (mp_car->engine ()->rotational_speed () < mp_car->engine ()->stall_speed ())
        mp_car->start_engine ();
      set_gas (0.0);

      m_state_time += time_step;
      if (m_state_time > 1.0)
        {
          set_brake (0.0);
          m_state_time = 0.0;
          m_state = STARTING;
        }
      return;

    case STARTING:
      if (m_state_time == 0.0)
        {
          mp_car->engage_clutch (3.0);
          mp_car->shift (1);
        }

      m_state_time += time_step;
      if (m_state_time > 3.0)
        {
          m_state_time = 0.0;
          m_state = DRIVING;
        }
      break;

    case DRIVING:
      break;
    }

  const Three_Vector track_position = 
    mp_track->track_coordinates (mp_car->chassis ().position (), 
                                 m_road_index, 
                                 m_segment_index);
  steer (track_position);
  shift ();
  accelerate (track_position);

  // Detect collisions last since we may override steering and
  // braking.
  if (m_interact)
    detect_collisions (track_position);
}

void
Robot_Driver::draw ()
{
  if (!m_show_steering_target)
    return;

  glLoadIdentity ();
  glPointSize (8.0);
  glBegin (GL_POINTS);

  // Draw where the car is currently pointed.
  const Three_Vector target (target_position ());
  glColor3d (0.0, 0.8, 0.0);
  glVertex3d (target.x, target.y, current_segment ().world_elevation (target) + 0.1);

  // Draw the point on the racing line.
  const Three_Vector goal (lane_shift (goal_position ()));
  glColor3d (8.0, 0.0, 0.0);
  glVertex3d (goal.x, goal.y, current_segment ().world_elevation (goal) + 0.1);

  glEnd ();
}

// The point that is kept on the racing line is this far ahead of the
// car.
double 
Robot_Driver::target_distance () const
{
  return 2.0 * mp_car->length () + 0.2 * speed ();
}

// The position of the point that the driver tries to keep on the racing line.
Three_Vector
Robot_Driver::target_position () const
{
  return mp_car->chassis ().transform_to_world 
    (mp_car->center () + Three_Vector (target_distance (), 0.0, 0.0));
}

Three_Vector
Robot_Driver::goal_position ()
{
  const Three_Vector ahead 
    = mp_track->track_coordinates (mp_car->center_position (), 
                                   m_road_index, 
                                   m_segment_index);
  return m_racing_line.target (ahead.x, target_distance ());
}

// Get the segment that the car is currently on.
const Gl_Road_Segment&
Robot_Driver::current_segment () const
{
  return *mp_track->get_road (m_road_index).segments ()[m_segment_index];
}

Three_Vector
Robot_Driver::lane_shift (const Three_Vector& target)
{
  const Road& road = mp_track->get_road (m_road_index);
  const Three_Vector track = road.track_coordinates (target, m_target_segment);

  const double across = m_lane_shift 
    * (m_lane_shift > 0.0
       ? road.racing_line ().left_width (road, track.x) - track.y
       : road.racing_line ().right_width (road, track.x) + track.y);

  const Gl_Road_Segment& segment = *road.segments ()[m_target_segment];
  double along = wrap (track.x, road.length ());
  Three_Vector world = road.position (along, track.y + across, segment);
  world.z = 0.0;
  return world;
}

void
Robot_Driver::steer (const Three_Vector& track_position)
{
  Three_Vector target = target_position () - mp_car->center_position ();
  target.z = 0.0;
  const Three_Vector line = goal_position ();
  const Three_Vector shifted = lane_shift (line);
  Three_Vector goal = shifted - mp_car->center_position ();
  goal.z = 0.0;

  m_steer_control.set (target.cross (goal).z);
  set_steering (m_steer_control.propagate (mp_car->steer_angle (), m_time_step));
 
  // Return to the racing line if shifting doesn't make much of a
  // difference.
  if ((line - shifted).magnitude () < mp_car->width ())
    m_lane_shift = 0.0;
}

void
Robot_Driver::shift ()
{
  if (m_state == STARTING)
    return;

  // Gear Selection
  int gear = mp_car->gear ();
  double omega = mp_car->engine ()->rotational_speed ();
  double up_omega = omega
    * mp_car->transmission ()->gear_ratio (gear + 1)
    / mp_car->transmission ()->gear_ratio (gear);
  double down2_omega = omega
    * mp_car->transmission ()->gear_ratio (gear - 2)
    / mp_car->transmission ()->gear_ratio (gear);

  double throttle = mp_car->engine ()->throttle ();
  double current_power = mp_car->engine ()->power (throttle, omega);
  double power = mp_car->engine ()->power (1.0, omega);
  double up_power = mp_car->engine ()->power (1.0, up_omega);
  double down2_power = mp_car->engine ()->power (1.0, down2_omega);

  m_shift_time += m_time_step;

  if (m_shift_time < 0.3)
    return;

  // Shift up if there's more power at the current revs in the next higher
  // gear. 
  if (up_power > power)
    {
      mp_car->shift_up ();
    }
  // Shift up if there's enough power in the next higher gear.
  else if ((up_power > current_power)
           && (up_power > 0.95 * power)
           && (throttle > 0.1)
           && (throttle < 0.9))
    {
      mp_car->shift_up ();
    }
  // Shift down if there's more power 2 gears down.
  else if ((down2_power > power)
           && (gear > 2))
    {
      mp_car->shift_down ();
    }

  if (mp_car->gear () != gear)
    m_shift_time = 0.0;

  return;
}

double
Robot_Driver::longitudinal_slip () const
{
  return abs_max (mp_car->wheel (0)->slip ().x,
                  mp_car->wheel (1)->slip ().x,
                  mp_car->wheel (2)->slip ().x,
                  mp_car->wheel (3)->slip ().x);
}

double
Robot_Driver::target_slip_angle () const
{
  return abs_max (mp_car->wheel (0)->peak_slip_angle (),
                  mp_car->wheel (1)->peak_slip_angle (),
                  mp_car->wheel (2)->peak_slip_angle (),
                  mp_car->wheel (3)->peak_slip_angle ());
}

double
Robot_Driver::total_slip () const
{
  return Three_Vector (longitudinal_slip (), transverse_slip (), 0.0).magnitude ();
}

double
Robot_Driver::slip_excess () const
{
  // Subtract from the slip vector a vector in the same direction that touches
  // the ellipse defined by the x- and y-components of the target slip vector.
  double r = total_slip ();
  // The direction is undefined for r=0.  Arbitrarily pick x.  
  if (r < 1.0e-9)
    return -m_target_slip;
  double x = longitudinal_slip ();
  double y = transverse_slip ();
  // The factor that relates the lengths of the slip and target vectors. 
  double c = m_target_slip / std::sqrt (x*x + y*y);
  return r * (1.0 - c);
}

double
Robot_Driver::transverse_slip () const
{
  return abs_max (mp_car->wheel (0)->slip ().y,
                  mp_car->wheel (1)->slip ().y,
                  mp_car->wheel (2)->slip ().y,
                  mp_car->wheel (3)->slip ().y);
}

double
Robot_Driver::speed () const
{
  return mp_car->chassis ().cm_velocity ().magnitude();
}

void
Robot_Driver::accelerate (const Three_Vector& track_position)
{
  double start = current_segment ().start_distance ();

  // Ignore kerbs when calculating the normal for the racing line speed.
  Three_Vector normal 
    = current_segment ().normal (track_position.x - start, track_position.y, false);

  const double drag = mp_car->chassis ().aerodynamic_drag ();
  const double lift = mp_car->chassis ().aerodynamic_lift ();

  double cornering_speed 
    = m_racing_line.maximum_speed (track_position.x, 
                                   m_lane_shift,
                                   lift,
                                   normal,
                                   mp_car->chassis ().mass ());

  double braking_speed
    = m_braking.maximum_speed (speed (),
                               track_position.x,
                               m_lane_shift,
                               drag,
                               lift,
                               mp_car->chassis ().mass ());

  // std::cerr << track_position.x << ' ' 
  //           << cornering_speed << ' ' 
  //           << braking_speed << ' '
  //           << speed () << std::endl;

  set_speed (std::min (cornering_speed, braking_speed));
}

void
Robot_Driver::set_speed (double max_speed)
{
  m_speed_control.set (max_speed);
  double d1 = m_speed_control.propagate (speed (), m_time_step);
  double d2 = m_traction_control.propagate (total_slip (), m_time_step);
  double gas = std::min (d1, d2);

  if (!mp_car->clutch ()->engaged ())
    {
      // Keep the revs in check if the clutch is not fully engaged.
      m_speed_control.set (0.0);
      double error = (mp_car->engine ()->rotational_speed ()
                      - mp_car->engine ()->peak_engine_speed ());
      gas = std::min (gas, m_speed_control.propagate (0.01 * error, m_time_step));
    }

  set_gas (gas);

  m_brake_control.set (std::min (max_speed, speed ()));
  double b1 = -m_brake_control.propagate (speed (), m_time_step);
  double b2 = m_traction_control.propagate (total_slip (), m_time_step);
  set_brake (std::min (b1, b2));
}

void
Robot_Driver::set_steering (double angle)
{
  const double max_angle = 1.5 * target_slip_angle ();
  mp_car->steer (clip (angle, -max_angle, max_angle), 0.0, true);
  // true => direct steering: Ignore non-linearity and speed-sensitivity.
}

void
Robot_Driver::set_gas (double gas)
{
  if (gas == 0.0)
    {
      m_speed_control.reset ();
      m_traction_control.reset ();
    }
  mp_car->gas (clip (gas, 0.0, 1.0));  
}

void
Robot_Driver::set_brake (double brake)
{
  if (brake == 0.0)
    m_brake_control.reset ();
  mp_car->brake (clip (brake, 0.0, 1.0));
}

// Check for potential collisions.
void
Robot_Driver::detect_collisions (const Three_Vector& track_position)
{
  if (mp_cars == 0) return;

  // Ignore cars that won't make contact for this many seconds.
  double minimum_crash_time = 10.0;
  double minimum_distance = 10.0 * mp_car->length ();
  double cross = 0.0;
  Three_Vector v2;

  const Road& road = mp_track->get_road (m_road_index);

  // Loop through the other cars.
  for (std::vector <Car_Information>::const_iterator it = mp_cars->begin ();
       it != mp_cars->end ();
       it++)
    {
      if (it->car == mp_car)
        continue;

      size_t segment = it->segment_index;
      size_t road_index = it->road_index;
      const Three_Vector other_track_position = 
        mp_track->track_coordinates (it->car->chassis ().position (), 
                                     road_index, 
                                     segment);

      if (!is_in_range (road.distance (other_track_position.x, track_position.x), 
                        -0.5 * mp_car->length (), 5.0 * mp_car->length ()))
        {
          // Ignore cars that are far away.
          continue;
        }

      const Three_Vector r1 = mp_car->chassis ().cm_position ();
      const Three_Vector v1 = mp_car->chassis ().cm_velocity ();
      const Three_Vector r2 = it->car->chassis ().cm_position ();
      v2 = it->car->chassis ().cm_velocity ();

      // The how close the other car will come if both move at
      // constant velocity.
      const double closest = closest_approach (r1, v1, r2, v2);
      const double closing = closing_speed (r1, v1, r2, v2);

      if ((closest < 3.0 * mp_car->length ()) && (closing > 0.0))
        {
          const Three_Vector delta_r = r2 - r1;
          const double distance = delta_r.magnitude ();
          minimum_crash_time = std::min (minimum_crash_time, distance / closing);
          minimum_distance = std::min (distance, minimum_distance);
          cross = v1.cross (delta_r).z;
        }
    }

  static const double shift_step = 1.0;
  // 2*length = 1 length between
  if ((minimum_crash_time < 3.0) || 
      (minimum_distance < 2.0 * mp_car->length ()))
    { // Avoid the collision.
      if ((slip_excess () < 0.0) || (std::abs (m_lane_shift) == 1.0))
        { // Can't swerve; slow down to other car's speed.
          set_speed (v2.magnitude ());

          // double delta_v = speed () - v2.magnitude ();
          // m_speed_control.set (
          // set_gas (0.0);
          // //!!anti-lock
          // m_brake_control.set (0.0);
          // set_brake (-m_brake_control.propagate (0.5 * delta_v, m_time_step));
        }
      else
        { // We have traction to spare; swerve.
          if (cross < 0.0)
            m_lane_shift = std::min (1.0, m_lane_shift + shift_step);
          else
            m_lane_shift = std::max (-1.0, m_lane_shift - shift_step);
        }
    }
  else
    { // No danger of collision.  Get back to the racing line.
      if (m_lane_shift > 0.0)
        m_lane_shift = std::max (0.0, m_lane_shift - shift_step);
      else if (m_lane_shift < 0.0)
        m_lane_shift = std::min (0.0, m_lane_shift + shift_step);
    }
}

//-----------------------------------------------------------------------------

// The distance resolution of the braking speed calculation
static const double delta_x = 10.0;
// Braking is applied gradually.  It reaches its maximum in this many meters.
static const double fade_in = 50.0;

Braking_Operation::Braking_Operation (const Road& road,
                                      double deceleration,
                                      double gravity,
                                      const Robot_Racing_Line& line)
  : m_start (0.0),
    m_length (0.0),
    m_is_braking (false),
    m_road (road),
    m_deceleration (deceleration),
    m_gravity (gravity),
    m_line (line)
{
}

Braking_Operation::~Braking_Operation ()
{
  // Do the proper cleanup if we were deleted during a braking operation.
  end ();
}

void
Braking_Operation::start (double start, double length)
{
  // Use start distance and length instead of start and end to avoid issues with
  // wrapping around the track.
  m_start = start;
  m_length = length;
  m_is_braking = true;
}

void
Braking_Operation::end ()
{
  m_is_braking = false;
}

bool
Braking_Operation::check_done_braking (double distance)
{
  if (past_end (distance))
    end ();
  return !m_is_braking;
}

double
Braking_Operation::distance_from_start (double distance) const
{
  if (distance >= m_start)
    return (distance - m_start);
  else // wrap around the track
    return (distance + m_road.length () - m_start);
}

bool
Braking_Operation::past_end (double distance) const
{
  return (distance_from_start (distance) > m_length);
}

double 
Braking_Operation::deceleration (const Three_Vector& curvature,
                                 double speed, 
                                 double drag,
                                 double lift,
                                 const Three_Vector& n_hat,
                                 const Three_Vector& p_hat,
                                 double mass,
                                 double fraction) const
{
  double c = curvature.magnitude ();
  double mu = m_deceleration * fraction;
  double v2 = speed * speed;
  Three_Vector r_hat = curvature.unit ();
  return (m_gravity*p_hat.z 
          - v2*drag/mass
          + mu * (m_gravity*n_hat.z - v2*(lift/mass + c*r_hat.dot (n_hat))));
}

Three_Vector
Braking_Operation::get_normal (double along) const
{
  const Road_Segment* segment = m_road.segment_at (along);
  double along_segment = along - segment->start_distance ();
  return segment->normal (along_segment, 0.0);
}

double
Braking_Operation::delta_braking_speed (double speed,
                                        double cornering_speed,
                                        double along,
                                        double lane_shift,
                                        const Three_Vector& normal,
                                        double drag, 
                                        double lift, 
                                        double mass,
                                        double fraction) const
{
  Three_Vector curvature = m_line.curvature (along, lane_shift);
  Three_Vector p = m_line.tangent (along);

  // std::cerr << along << ' '
  //           << m_road.elevation ().interpolate (along) << ' '
  //           << curvature.magnitude () << ' '
  //           << p.z << std::endl;
 

  double a = deceleration (curvature, speed, drag, lift, normal, p, mass, fraction);
  double a_par = a * (1.0 - speed / cornering_speed);
  return a_par * delta_x / speed;
}

double 
Braking_Operation::maximum_speed (double speed,
                                  double distance,
                                  double lane_shift,
                                  double drag,
                                  double lift,
                                  double mass)
{
  // Return the maximum safe speed under braking.

  // See if we've past the end of a braking operation.
  check_done_braking (distance);

  // If we're in the middle of a braking operation, get the speed from the speed
  // vs. braking curve.
  if (is_braking ())
    {
      if (distance < m_speed_vs_distance [0].x)
        distance += m_road.length ();
      return m_speed_vs_distance.interpolate (distance);
    }

  // Calculate the car's speed as a function of distance if braking started now.
  // If the projected speed exceeds the maximum cornering speed calculated from
  // the racing line then braking should start now.  When this happens, find the
  // minimum cornering speed and calculate distance vs. speed backwards from
  // there. 

  Two_Vector minimum_speed_vs_distance (0.0, speed);

  // True if projected braking speed exceeds cornering speed anywhere.
  bool too_fast = false;
  // True if a minimum in the cornering speed was found in the distance range
  // where projected braking speed exceeds cornering speed.
  bool found_min = false;
  double start_speed = speed;
  // Look up to 1000 m ahead.
  for (double d = 0.0; d < 1000.0; d += delta_x)
    {
      double along = wrap (distance + d, m_road.length ());
      Three_Vector normal = get_normal (along);
      double cornering_speed 
        = m_line.maximum_speed (along, lane_shift, lift, normal, mass);

      // Apply braking gradually.
      double braking_fraction = std::min (d / fade_in, 1.0);
      double dv = delta_braking_speed (speed, cornering_speed, along, lane_shift, 
                                       normal, drag, lift, mass, braking_fraction);

      speed -= dv;
      if (speed <= 0.0)
        break;

      if (speed >= cornering_speed)
        {
          // Already too fast, nothing we can do.
          if (d == 0.0)
            break;
          too_fast = true;
        }
      else if (too_fast)
        {
          // We've gone from a region where braking speed is higher than
          // cornering speed to one where it's lower.  Keep going in case
          // there's another curve up ahead that requires harder braking.
          found_min = true;
          too_fast = false;
        }

      if (too_fast && (cornering_speed < minimum_speed_vs_distance.y))
        minimum_speed_vs_distance = Two_Vector (d, cornering_speed);
    }

  // No need to start braking yet.
  if (!found_min)
    return std::numeric_limits <double>::max ();

  // Build the speed vs. distance curve by working backwards from the minimum
  // speed.  Start one interval beyond the end; end one interval before the
  // beginning to ensure that the interpolations are good slightly beyond the
  // endpoints.
  too_fast = false;
  std::vector <Two_Vector> points;
  speed = minimum_speed_vs_distance.y;
  for (double d = minimum_speed_vs_distance.x; d > -delta_x; d -= delta_x)
    {
      // Use un-wrapped distances so the interpolator's points are in order. 
      points.push_back (Two_Vector (distance + d, speed));

      double along = wrap (distance + d, m_road.length ());
      Three_Vector normal = get_normal (along);
      double cornering_speed 
        = m_line.maximum_speed (along, lane_shift, lift, normal, mass);

      double braking_fraction = std::min (d / fade_in, 1.0);
      double dv = delta_braking_speed (speed, cornering_speed, along, lane_shift, 
                                       normal, drag, lift, mass, braking_fraction);

      if (too_fast && (cornering_speed < minimum_speed_vs_distance.y))
        minimum_speed_vs_distance = Two_Vector (d, cornering_speed);

      if (speed >= cornering_speed)
        {
          if (!too_fast)
            {
              minimum_speed_vs_distance = Two_Vector (d, cornering_speed);
              // Found an earlier curve that requires a lower speed.
              too_fast = true;
            }
        }
      else if (too_fast)
        {
          // Found the new minimum.  Start over from there.
          d = minimum_speed_vs_distance.x;
          speed = minimum_speed_vs_distance.y;
          points.clear ();
          points.push_back (minimum_speed_vs_distance 
                            + Two_Vector (distance + delta_x, 0.0));
          too_fast = false;
        }
      else
        speed += dv;
    }

  // The interpolator requires ascending x-values.  Reverse the points.
  m_speed_vs_distance.clear ();
  std::reverse (points.begin (), points.end ());
  m_speed_vs_distance.load (points);

  // Scale speed vs. distance so it matches the passed-in speed at the passed-in
  // distance.  This is usually a small adjustment, but can be significant when
  // curves are closely spaced.
  double delta_speed = start_speed - m_speed_vs_distance [0].y;
  for (size_t i = 0; i < m_speed_vs_distance.size (); i++)
    {
      double fraction 
        = ((distance + minimum_speed_vs_distance.x - m_speed_vs_distance [i].x)
           / (distance + minimum_speed_vs_distance.x - m_speed_vs_distance [0].x));
      m_speed_vs_distance [i].y += fraction * delta_speed;

      // std::cerr << m_speed_vs_distance [i].x << ' ' 
      //           << m_speed_vs_distance [i].y << ' ' << fraction << std::endl;
    }

  // Mark the start of a braking operation.
  start (distance, minimum_speed_vs_distance.x);
  // No need to restrict speed yet.  Next call will get it from the newly
  // calculated speed vs. distance curve.
  return std::numeric_limits <double>::max ();
}

//-----------------------------------------------------------------------------
Robot_Racing_Line::Robot_Racing_Line (const Road& road,
                                      double lateral_acceleration,
                                      double gravity)
  : mp_road (&road),
    m_lateral_acceleration (lateral_acceleration),
    m_gravity (gravity)
{
}

Three_Vector
Robot_Racing_Line::curvature (double along, double lane_shift) const
{
  return mp_road->racing_line ().curvature (along, lane_shift);
}

Three_Vector
Robot_Racing_Line::tangent (double along) const
{
  return mp_road->racing_line ().tangent (along);
}

double
Robot_Racing_Line::maximum_speed (double distance, 
                                  double lane_shift,
                                  double lift,
                                  const Three_Vector& n_hat,
                                  double mass) const
{
  const Three_Vector curve = curvature (distance, lane_shift);
  double c = curve.magnitude ();
  double mu = m_lateral_acceleration;
  Three_Vector r_hat = curve.unit ();
  Three_Vector r_perp (-r_hat.y, r_hat.x, 0.0);
  Three_Vector q_hat = n_hat.rotate (0.5 * pi * r_perp.unit ());

  double upper = m_gravity*(q_hat.z + mu*n_hat.z);
  double lower = c*(r_hat.dot(q_hat) + mu*r_hat.dot(n_hat)) + mu*lift/mass;

  if (lower > 1e-9)
    return std::sqrt (upper / lower);
  else
    return std::numeric_limits <double>::max ();
}

Three_Vector
Robot_Racing_Line::target (double along, double lead) const
{
  return mp_road->racing_line ().position (along + lead);
}
