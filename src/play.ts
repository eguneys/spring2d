import Input from './input'
import { Transform, Vec2, Quad } from 'soli2d'
import { RigidSteer } from './rigid'

let Template = new Transform()

let stage: Transform;
let input: Input;
let mouse: Mouse;
let image: HTMLImageElement;

let updates = []

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
    this.player = make_player(this)
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

  updates.push((dt, dt0) => {
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


export function make_player(gg: GG) {

  let t_base = Template.clone
  
  let bg = anim(0, 4, 1, 1, t_base)
  bg.size = Vec2.make(2, 2)

  t_base._set_parent(stage)

  let steer = RigidSteer.make(32, 32, {
    mass: 1000,
    air_friction: 0.8,
    max_speed: 6,
    max_force: 0.1 
  })

  //steer.v_evosion = Vec2.make(0, 0)
  steer.v_wander = Vec2.make(0, 0)

  updates.push((dt, dt0) => {
   // steer.v_evosion.set_in(gg.cursor.x, gg.cursor.y)
    steer.update(dt, dt0)

    t_base.x = steer.x
    t_base.y = steer.y

  })
}


