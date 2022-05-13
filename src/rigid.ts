import { Vec2 } from 'soli2d'

export type Rigid = {
  mass: number,
  air_friction: number,
  max_speed: number,
  max_force: number,
  force: number,
  x: number,
  x0: number,
  vx: number
}

export function make_rigid(x, mass, air_friction, max_speed, max_force) {
  return {
    max_force,
    max_speed,
    air_friction,
    mass,
    force: 0,
    x,
    x0: x,
    vx: 0
  }
}

export function rigid_update(body: Rigid, dt: number, dt0: number) {

  let { air_friction, force, mass, max_speed, max_force } = body

  let { x, x0 } = body

  let a = force / mass

  a = Math.min(a, max_force)
  let v0_x = x - x0
  let new_vx = v0_x * air_friction * dt / dt0 + a * dt * (dt + dt0) / 2
  new_vx = Math.min(new_vx, max_speed)
  let new_x0 = x,
    new_x = x + new_vx


  body.x0 = new_x0
  body.x = new_x
  body.vx = new_vx
}

export type RigidOptions = {
  mass: number,
  air_friction: number,
  max_speed: number,
  max_force: number
}

export class RigidSteer {

  static make = (x: number, y: number, opts: RigidOptions) => {
    return new RigidSteer(Vec2.make(x, y), opts)
  }

  readonly r_x: Rigid
  readonly r_y: Rigid

  v_seek?: Vec2
  v_arrive?: Vec2

  v_pursuit?: Vec2
  v0_pursuit?: Vec2

  get x() { return this.r_x.x }
  get y() { return this.r_y.x }

  get pos() { 
    this.vs.set_in(this.x, this.y)
    return this.vs
  }


  v_vx: Vec2

  get velocity() {
    this.v_vx.set_in(this.r_x.vx,
                this.r_y.vx)
    return this.v_vx
  }

  get max_speed() {
    return this.opts.max_speed
  }

  constructor(readonly vs: Vec2,
              readonly opts: RigidOptions) {

                this.v_vx = Vec2.zero

                this.r_x = make_rigid(vs.x,
                                    opts.mass,
                                    opts.air_friction, 
                                    opts.max_speed, 
                                    opts.max_force)


                this.r_y = make_rigid(vs.y,
                                    opts.mass,
                                    opts.air_friction, 
                                    opts.max_speed, 
                                    opts.max_force)
              }

    update(dt: number, dt0: number) {


      if (this.v_seek) {
        let desired_vel = seek_steer(this.pos, this.v_seek, this.max_speed)
        let steering = desired_vel.sub(this.velocity)

        this.r_x.force = steering.x
        this.r_y.force = steering.y
      }

      if (this.v_arrive) {
        let desired_vel = arrive_steer(this.pos, this.v_arrive, this.max_speed, 16)
        let steering = desired_vel.sub(this.velocity)
        this.r_x.force = steering.x
        this.r_y.force = steering.y
      }

      if (this.v_pursuit) {
        let desired_vel = pursuit_steer(this.pos, this.v_pursuit, this.max_speed, this.v0_pursuit?this.v_pursuit.sub(this.v0_pursuit):Vec2.zero)
        let steering = desired_vel.sub(this.velocity)
        this.r_x.force = steering.x
        this.r_y.force = steering.y
        if (!this.v0_pursuit) {
          this.v0_pursuit = this.v_pursuit.clone
        } else {
          this.v0_pursuit.set_in(this.v_pursuit.x, this.v_pursuit.y)
        }
      }

      if (this.v_evosion) {
        let desired_vel = evosion_steer(this.pos, this.v_evosion, this.max_speed, this.v0_evosion?this.v_evosion.sub(this.v0_evosion):Vec2.zero)
        let steering = desired_vel.sub(this.velocity)
        this.r_x.force = steering.x
        this.r_y.force = steering.y
        if (!this.v0_evosion) {
          this.v0_evosion = this.v_evosion.clone
        } else {
          this.v0_evosion.set_in(this.v_evosion.x, this.v_evosion.y)
        }
      }

      if (this.v_wander) {
        let desired_vel = wander_steer(this.pos, this.v_wander, 1, 2, 2)
        let steering = desired_vel.sub(this.velocity)
        this.r_x.force = steering.x
        this.r_y.force = steering.y
      }

      rigid_update(this.r_x, dt, dt0)
      rigid_update(this.r_y, dt, dt0)
    }
}

export function wander_steer(position: Vec2, wander_target: Vec2, jitter: number, r: number, distance: number) {
  wander_target.x += (1 - 2 * Math.random()) * jitter
  wander_target.y += (1 - 2 * Math.random()) * jitter 
  let transform = wander_target.normalize.scale_in(r).add_in(Vec2.make(distance, 0))
  console.log(wander_target.normalize.scale_in(r))
  return transform
}

export function pursuit_steer(position: Vec2, target: Vec2, max_speed: number, target_vel: Vec2) {
  let D = position.distance(target)
  let c = 0.08
  let T = D * c
  let prediction = target.add(target_vel.normalize.scale(T))
  return seek_steer(position, prediction, max_speed)
}

export function evosion_steer(position: Vec2, target: Vec2, max_speed: number, target_vel: Vec2) {
  let D = position.distance(target)
  let c = 0.08
  let T = D * c
  let prediction = target.add(target_vel.normalize.scale(T))
  return flee_steer(position, prediction, max_speed)
}

export function seek_steer(position: Vec2, target: Vec2, max_speed: number) {
  return target.sub(position).normalize.scale_in(max_speed)
}

export function flee_steer(position: Vec2, target: Vec2, max_speed: number) {
  return position.sub(target).normalize.scale_in(max_speed)
}

export function arrive_steer(position: Vec2, target: Vec2, max_speed: number, slowing_distance: number) {
  let target_offset = target.sub(position)
  let distance = target_offset.length
  if (distance === 0) {
    return Vec2.zero
  }
  let ramped_speed = max_speed * (distance / slowing_distance)
  let clipped_speed = Math.min(ramped_speed, max_speed)
  let desired_velocity = target_offset.scale_in(clipped_speed / distance) 
  return desired_velocity
}

