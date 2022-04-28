import Input from './input'
import { Transform, Vec2, Quad } from 'soli2d'

let Template = new Transform()

let stage: Transform;
let input: Input;
let image: HTMLImageElement;

export function anim(x: number, y: number, w: number, h: number, parent: Transform = stage) {
  let bg = Template.clone
  bg.quad = Quad.make(image, x, y, w, h)
  parent && bg._set_parent(parent)
  return bg
}

export function _init(_stage: Transform, _image: HTMLImageElement, _input: Input) {
  stage = _stage
  input = _input
  image = _image

  let bg = anim(2, 2, 1, 1)
  bg.size = Vec2.make(64, 64)



}


export function _update(dt: number, dt0: number) {


}
