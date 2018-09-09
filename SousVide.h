#ifndef RECIPES_H_

#define RECIPES_H_

#define NUMBER_OF_RECIPES 4

struct recipe recipes[NUMBER_OF_RECIPES] = {
  {
    "STEAK", {
      { true, "HEATING", "CHECK WATER", "", 54.4, 0 },
      { true, "COOKING", "ADD STEAK", "REMOVE STEAK", 54.4, 120 },
      { true, "SOUS VIDE", "", "COMPLETE!", 0, 0 },
      { false, "", "", "", 0, 0 },
      { false, "", "", "", 0, 0 },
      { false, "", "", "", 0, 0 }
    },
    { -1, -1, -1 }
  },
  {
    "60 MIN EGG", {
      { true, "HEATING", "CHECK WATER", "", 63.0, 0 },
      { true, "COOKING", "ADD EGG(S)", "REMOVE EGG(S)", 63.0, 60 },
      { true, "SOUS VIDE", "", "COMPLETE!", 0, 0 },
      { false, "", "", "", 0, 0 },
      { false, "", "", "", 0, 0 },
      { false, "", "", "", 0, 0 }
    },
    { -1, -1, -1 }
  },
  {
    "SALMON MEDRARE", {
      { true, "HEATING", "CHECK WATER", "", 52.0, 0 },
      { true, "COOKING", "ADD SALMON", "REMOVE SALMON", 52.0, 20 },
      { true, "SOUS VIDE", "", "COMPLETE!", 0, 0 },
      { false, "", "", "", 0, 0 },
      { false, "", "", "", 0, 0 },
      { false, "", "", "", 0, 0 }
    },
    { -1, -1, -1 }
  },
  {
    "SALMON MEDIUM", {
      { true, "HEATING", "CHECK WATER", "", 57.0, 0 },
      { true, "COOKING", "ADD SALMON", "REMOVE SALMON", 57.0, 20 },
      { true, "SOUS VIDE", "", "COMPLETE!", 0, 0 },
      { false, "", "", "", 0, 0 },
      { false, "", "", "", 0, 0 },
      { false, "", "", "", 0, 0 }
    },
    { -1, -1, -1 }
  }
};

#endif
