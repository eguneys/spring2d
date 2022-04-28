import { Soli2d, loop } from 'soli2d'
import sprites from '../assets/sprites.png'
import Input from './input'
import { _init, _update } from './play'


function load_image(path: string) {
  return new Promise(resolve => {
    let res = new Image()
    res.onload = () => resolve(res)
    res.src = path
  })
}

export default function app(element: HTMLElement) {

  load_image(sprites).then(image => {
    let [render, stage, $canvas] = Soli2d(element, image, 64, 64)

    let input = new Input().init()

    _init(stage, image, input)

    loop((dt, dt0) => {
      input.update(dt, dt0)
      _update(dt, dt0)
      stage._update_world()
      render()
    })
  })


}
