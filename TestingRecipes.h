#ifndef RECIPES_H_

#define RECIPES_H_

#define NUMBER_OF_RECIPES 3

struct recipe recipes[NUMBER_OF_RECIPES] = {
  {
    "Test 1", {
      { true, "HEAT", "ADD WATER", "", 35.0, 0 },
      { true, "MASH IN", "ADD MALT", "", 40.0, 0 },
      { true, "PROTEIN REST", "", "", 40.0, 0 },
      { true, "MALTOSE REST", "", "", 43.0, 0 },
      { true, "SACCRIFICATION", "", "REMOVE MALT", 47.0, 0 },
      { true, "HOPS BOIL", "", "COMPLETE!", 50, 2 }
    },
    { 2, 1, NULL }
  },
  {
    "Test 2", {
      { true, "HEAT", "ADD WATER", "", 35.0, 0 },
      { true, "MASH IN", "ADD MALT", "", 40.0, 0 },
      { true, "PROTEIN REST", "", "", 40.0, 0 },
      { true, "MALTOSE REST", "", "", 43.0, 0 },
      { true, "SACCRIFICATION", "", "REMOVE MALT", 47.0, 0 },
      { true, "HOPS BOIL", "", "COMPLETE!", 50, 2 }
    },
    { 2, 1, NULL }
  },
  {
    "Test 3", {
      { true, "HEAT", "ADD WATER", "", 35.0, 0 },
      { true, "MASH IN", "ADD MALT", "", 40.0, 0 },
      { true, "PROTEIN REST", "", "", 40.0, 0 },
      { true, "MALTOSE REST", "", "", 43.0, 0 },
      { true, "SACCRIFICATION", "", "REMOVE MALT", 47.0, 0 },
      { true, "HOPS BOIL", "", "COMPLETE!", 50, 2 }
    },
    { 2, 1, NULL }
  }
};

#endif
