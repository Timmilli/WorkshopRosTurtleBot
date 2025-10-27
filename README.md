# WorkshopRosTurtleBot

## Students

Quentin Fallito
Guénaël Roger

# Execute

Before everything:

```
colcon build
```

## Exercices 1 to 4

Then, to execute the script of a question X:

```
ros2 run publisher [talkerX|listenerX]
```
(for the 3rd question, replace `talkerX` with `talker3`)

## Exercice 5

For the talker:
```
ros2 run publisher talker5
```

For the listeners:
```
ros2 run publisher [listerner51|listener52]
```

## Exercice 6

For the talker:
```
ros2 launch publisher publisher_member_function_q6_launch.py
```

For the listeners:
```
ros2 run publisher listerner6
```

## Exercice 7

```
ros2 run publisher service
```
```
ros2 run py_srvcli client <computer_name> <domain_id>
```
Correct question: circuit, 2

## Exercice 8

```
ros2 launch launch/simple_launch.py
```

## Exercice 9

```
ros2 launch launch/param_launch.py
```
