# Media

Photos and short videos of the setup and of the demos.

Suggested files (keep each GIF/video small, ideally < 10 MB):

| File | Content |
|---|---|
| `setup.jpg` | the Touch connected to the laptop |
| `touch_demo.gif` | the stylus moving to targets published with `ros2 topic pub` |
| `usb_capture.png` | screenshot of the USB analysis (motor values always 2048) |

To show a GIF in the main README, add for example:

```markdown
![Touch demo](docs/media/touch_demo.gif)
```

To make a GIF from a phone video (example):
`ffmpeg -i demo.mp4 -vf "fps=10,scale=640:-1" -t 15 touch_demo.gif`
