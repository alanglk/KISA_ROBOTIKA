#ifndef WAYPOINTS_INCLUDE_GUARD_HPP
#define WAYPOINTS_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for generating control (body twist) between waypoints

#include <cmath>
#include <iosfwd>
#include <vector>

#include "rigid2d.hpp"
#include "diff_drive.hpp"


namespace rigid2d
{
  class Waypoints
  {
  public:
    /// \brief create a waypoints object for developing contols
    /// \param pts - list of waypoints to travel to
    /// \param rot_vel - roation velocity of turtle
    /// \param trans_vel - translation velocity of turtle
    Waypoints(const std::vector<Vector2D> &way_pts, double rot_vel, double trans_vel);

    /// \brief Sets the control gain
    /// \param k_rot - gain for rotation
    /// \param k_trans - gain for translation
    void setGain(double k_rot);

    /// \brief compose velocity for current waypoint
    /// \param pose - pose of robot in world coordinates
    /// \return cmd twist in the body frame
    Twist2D nextWaypoint(Pose pose);

    /// \brief compose velocity for current waypoint using closed loop feedback
    /// \param pose - pose of robot in world coordinates
    /// \return cmd twist in the body frame
    Twist2D nextWaypointClosedLoop(Pose pose);

  private:
    /// \brief increments the waypoint index
    void incrementWaypoint();

    /// \brief increments waypoint index if waypoint is reached
    /// \param pose - pose of robot in world coordinates
    void waypointReached(Pose pose);

    /// \brief compose distance from robot to waypoint
    /// \param pose - pose of robot in world coordinates
    double waypointDistance(Pose pose) const;

    /// \brief compose difference in heading of robot to waypoint
    /// \param pose - pose of robot in world coordinates
    /// \return difference in heading
    double waypointHeading(Pose pose) const;

    // DiffDrive drive;             // diff_drive object
    std::vector<Vector2D> pts;      // store waypoints
    int idx;                        // index of waypoint currently headed towards
    int ctr;                        // count total waypoints visited
    double htol, ptol;              // tolerances for heading and position
    double rot_vel, trans_vel;      // rotation/translatin velocities
    double k_rot;                   // rotation gain
    double k_trans;                 // translation gain
    bool cycle_complete;            // one cycle to all waypoints complete
  };






} // end namespace

#endif
