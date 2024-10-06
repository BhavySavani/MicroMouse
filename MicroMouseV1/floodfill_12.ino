
void wallcheck(int x, int y) {
  up = false, down = false, left = false, right = false;
  if ((HorizontalWalls[x][y] == 0)) {
    if (x == 0) {
      up = true;
    }
    else {
      if (manhattan_distances[x - 1][y] == 999) {
        up = true;
      }
    }

  }
  if ((HorizontalWalls[x + 1][y] == 0)) {
    if (x == 11) {
      down = true;
    }
    else {
      if (manhattan_distances[x + 1][y] == 999) {
        down = true;
      }
    }
  }
  if ((VerticalWalls[x][y] == 0)) {
    if (y == 0) {
      left = true;
    }
    else {
      if (manhattan_distances[x][y - 1] == 999) {
        left = true;
      }
    }
  }
  if ((VerticalWalls[x][y + 1] == 0)) {
    if (y == 11) {
      right = true;
    }
    else {
      if (manhattan_distances[x][y + 1] == 999) {
        right = true;
      }
    }
  }
}

void flood_fill() {
  for (int i =0;i<maze_size;i++){
    for (int j =0; j<maze_size;j++){
      manhattan_distances[i][j] = 999;
    }
  }

  int x = (maze_size / 2) - 1;
  int y = (maze_size / 2) - 1;
  for (int i = 0; i < 2; i++) {
    y = (maze_size / 2) - 1;

    for (int j = 0; j < 2; j++) {
      manhattan_distances[x][y] = 0;
      element.enqueue(0);
      elementX.enqueue(x);
      elementY.enqueue(y);
      y += 1;
    }
    x += 1;
  }

  while (!element.isEmpty()) {
    int current_element = element.dequeue();
    int current_elementX = elementX.dequeue();
    int current_elementY = elementY.dequeue();

    wallcheck(current_elementX, current_elementY);
    if (up == true)
    {
      manhattan_distances[current_elementX - 1][current_elementY] = current_element + 1;
      element.enqueue(current_element + 1);
      elementX.enqueue(current_elementX - 1);
      elementY.enqueue(current_elementY);
    }
    if (down == true)
    {
      manhattan_distances[current_elementX + 1][current_elementY] = current_element + 1;
      element.enqueue(current_element + 1);
      elementX.enqueue(current_elementX + 1);
      elementY.enqueue(current_elementY);
    }
    if (right == true)
    {
      manhattan_distances[current_elementX][current_elementY + 1] = current_element + 1;
      element.enqueue(current_element + 1);
      elementX.enqueue(current_elementX);
      elementY.enqueue(current_elementY + 1);

    }
    if (left == true)
    {
      manhattan_distances[current_elementX][current_elementY - 1] = current_element + 1;
      element.enqueue(current_element + 1);
      elementX.enqueue(current_elementX);
      elementY.enqueue(current_elementY - 1);

    }
  }
  for (int i = 0;i<maze_size;i++){
    for (int j = 0;j<maze_size;j++){
      Serial.print(manhattan_distances[i][j]);
    }
    Serial.println();
  }

}
