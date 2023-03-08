## SNAKE 3D-Final Project:

### Game Purpose:
The player needs to collect all red object(apples, each one gives the sanke another joint)
in every level while avoiding black obstacles(bombs, if you hit one - you lose)
There are two more objects you can collect. coins for extra points and green balls for limitted time speedup.
every level begin with the space key and end when collected all apples or hit a bomb.
the difficulty is increasing with every level, you must eat more apples while avoidin

### GUI:
We have a menu screen with a few buttons and a highscore system.
In game we have an overlay for showing the current score and an option to change between the cameras.
In addition, there is an option to pause and unpause the game.
Once you pass to the next level you see your current score and a button to continue, if you beat level 3 you see a You Win! screen.
If you lose you see a You Lose! screen.

### Background:
We have added more textures (cubemaps) for the different levels.
The first level has the regular cubemap, the second and third are with new cubemaps.  
-- It was hard to add the cubemaps due to the CMake

### Collision Detection:
We've used the algo from assignment 2 to detect collisions between the snake and the objects.
Since we have a lot of objects we make sure we only check for collisions when the snake is within a certain distance from an object.
Another way to optimise this was to skip a few frames every time we detect a collision, so every 5 frames we only run the function once.
We have 4 obstacles (apple, bomb, coin and ball), all are implementing an interface called GameObject.
In GameObject we have a function that handles the logic of the collision for every type of object.
-- Mainly optimization

### Movement:
Our snake moves in the 3d world space with translation. The player can move the snake with the arrow keys.
We have a chain of links set to follow the head joint in the correct angle and smooth movement (snake like).
The rest of the objects are moving in a random direction.  
-- it was difficult to make the movement work correctly with the different camera and joints to work.

### Sound:
There are sound effects for hitting a bomb, clicking buttons, eating an apple and picking up coins or a ball.
In addition, there is a button to turn background music on and off. 

### Optimization:
Collision detection and we've predecimated some models with the code from assignment 1 (we have saved it as a file to avoid decimated it each time)

### Texture Mapping:
We have added textures to all objects. we've mapped the UV coords for the objects and attached textures to them.  
-- it was hard to work with closed models because igl::boundary_loop will not work on them.

### Cameras:
We have 3 cameras.
static from the top, snake view (child of the snake - looking from it's eyes), and 3rd person snake camera.

### Highscore:
We've added highscore to the game to keep it interesting. it keeps track of your best score between game sessions.

### Fullscreen support:
The game changes the screen size dynamically to support every kind of screen size. 


### Difficulties:
- CMake
- Efficient collision detection
- Snake movement
- Texture mapping
- Simplification and optimization
- Random movement of objects
- Optimization
