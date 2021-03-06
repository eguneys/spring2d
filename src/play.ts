import { ticks } from './shared'
import Input from './input'
import { Transform, Vec2, Quad } from 'soli2d'
import { RigidSteer } from './rigid'

let Template = new Transform()

let stage: Transform;
let input: Input;
let mouse: Mouse;
let image: HTMLImageElement;

let updates = []

function unbindable_update(fn: (dt: number, dt0: number) => void) {
  updates.push(fn)
  return () => {
    updates.splice(updates.indexOf(fn), 1)
  }
}

function dispose(a: Array<() => void>) {
  a.forEach(_ => _())
  a.length = 0
}

export function anim(x: number, y: number, w: number, h: number, parent: Transform = stage) {
  let bg = Template.clone
  bg.quad = Quad.make(image, x, y, w, h)
  parent && bg._set_parent(parent)
  return bg
}

export class GG {


  cursor: Cursor
  player: Player

  constructor() {
    this.cursor = make_cursor(this)

    let bg = anim(0, 0, 1, 1)
    bg.size = Vec2.make(1, 1)
    bg.x = 32
    bg.y = 32

    bg._set_parent(stage)


    let d_ps = []
    d_ps.push(make_player(this))

    unbindable_update((dt, dt0) => {
      if (input.btnp('z')) {
        dispose(d_ps)
        for (let i = 0; i < 10; i++) {
          d_ps.push(make_player(this))
        }
      }
    })
  }
}

export function _init(_stage: Transform, _image: HTMLImageElement, _input: Input, _mouse: Mouse) {
  stage = _stage
  input = _input
  image = _image
  mouse = _mouse

  let bg = anim(2, 2, 1, 1)
  bg.size = Vec2.make(64, 64)


  let gg = new GG()

}


export function _update(dt: number, dt0: number) {
  updates.forEach(_ => _(dt, dt0))
}

export function make_cursor() {

  let t_base = Template.clone
  
  let bg = anim(0, 0, 1, 1, t_base)
  bg.size = Vec2.make(1, 1)

  let bg2 = anim(0, 0, 1, 1, t_base)
  bg2.size = Vec2.make(1, 1)
  bg2.x = -1
  bg2.y = -1

  t_base._set_parent(stage)

  let _x = 0,
    _y = 0

  let d_u = unbindable_update((dt, dt0) => {
    let { hover } = mouse
    if (hover) {
      _x = hover[0]
      _y = hover[1]
    }

      t_base.x = _x
      t_base.y = _y
  })


  return {
    get x() { return _x },
    get y() { return _y }
  }
}

export function make_line(_x: number, _y: number, x: number, y: number) {

  let t_base = Template.clone

  let dir = Vec2.make(x, y).normalize

  for (let i = 0; i < 3; i++) {
    let _d = dir.scale(i)
    let bg = anim(0, 4, 1, 1, t_base)
    bg.size = Vec2.unit
    bg.x = _d.x
    bg.y = _d.y
  }

  t_base._set_parent(stage)
  t_base.x = _x
  t_base.y = _y

  return () => {
    t_base._remove()
  }
}

let ps = []

export function make_player(gg: GG) {

  let t_base = Template.clone
  
  let bg = anim(0, 4, 1, 1, t_base)
  bg.size = Vec2.make(2, 2)

  t_base._set_parent(stage)

  let steer = RigidSteer.make(32000, 32000, {
    mass: 1000,
    air_friction: 0.8,
    max_speed: 800,
    max_force: 200
  })

  steer.v_evosion = Vec2.make(0, 0)
  steer.v_wander = Vec2.make(0,0)
  steer.v_pursuit = Vec2.make(0, 0)

  let i = 0

  let t_l = []
  let d_u = unbindable_update((dt, dt0) => {

    steer.v_separation = ps.filter(_ => _!== steer).map(_ => _.matrix)
    if ((i+= dt) > ticks.seconds* 4) {

      if (steer.v_pursuit) {
        steer.v_pursuit = undefined
      } else {
        steer.v_pursuit = Vec2.make(0, 0)
      }
      i = 0
    }

    steer.v_evosion.set_in(gg.cursor.x * 1000, gg.cursor.y*1000)
    steer.v_pursuit?.set_in(gg.cursor.x * 1000, gg.cursor.y*1000)
    steer.update(dt, dt0)

    t_base.x = steer.x/ 1000
    t_base.y = steer.y/ 1000

    dispose(t_l)

    t_l.push(make_line(steer.x/1000, steer.y/1000, steer.side.x/1000, steer.side.y/1000))
  })

  ps.push(steer)

  return () => {
    d_u()
    dispose(t_l)
    t_base._remove()
    ps.splice(ps.indexOf(steer), 1)
  }
}


