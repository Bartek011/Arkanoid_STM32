

<img src="https://github.com/Bartek011/Arkanoid_STM32/assets/133026728/3490c9a5-0303-43db-a9ba-7a6b390312b8" alt="real" width="1000" height="300" style="display: block; margin: 0 auto;\">

The project was created as a summary of the class "Programming of embedded systems" (pol. "Programowanie Systemów Wbdudowanych" - PSW) at the Warsaw University of Technology. The main aim was to recreate retro game <a href="https://en.wikipedia.org/wiki/Arkanoid" target="_blank">Arkanoid</a> using:
- Development board STM32 NUCLEO-L476RG,
- LCD display from Nokia 5110,
- two analog joysticks
# Gameplay

After boot game allow us to choose:
- difficulty level as speed of the ball
- type of map:
  - static,
  - randomized after every bounce,
  - "chessboard"
- number of players.
- display actual record
  ## Rules
Once the game has been selected, the player must pass all the levels (up to speed level 3, the fastest) without letting the ball fall below the bottom edge of the screen.

There are special types of blocks that require more knockdowns and bonus blocks that change the length of the player-controlled platform for a set period of time.
## Video
<a href="https://www.youtube.com/watch?v=pgBTojNFUu4" target="_blank"><img src="https://i.imgur.com/fNSnsYo.png" alt="Gameplay"></a>
# Scheme

<img src="https://github.com/Bartek011/Arkanoid_STM32/assets/133026728/b84dd0e1-2674-486a-8c3e-ab5c631655a6" alt="real" width="1138" height="526" style="display: block; margin: 0 auto;\">

[schemat.pdf](https://github.com/Bartek011/Arkanoid_STM32/files/12411500/schemat.pdf)




