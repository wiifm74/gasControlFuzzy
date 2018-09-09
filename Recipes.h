#ifndef RECIPES_H_

#define RECIPES_H_

#define NUMBER_OF_RECIPES 10

  struct recipe recipes[NUMBER_OF_RECIPES] = {
  {
    "Festival Beer", {
      { true, "HEAT", "ADD WATER", "", 60.0, 0 },
      { true, "MASH IN", "ADD MALT", "", 63.0, 0 },
      { true, "PROTEIN REST", "", "", 63.0, 25 },
      { true, "MALTOSE REST", "", "", 73.0, 40 },
      { true, "SACCRIFICATION", "", "REMOVE MALT", 78.0, 10 },
      { true, "HOPS BOIL", "", "COMPLETE", 99.5, 80 }
    },
    { 70, 10, 0 }
  },
  {
    "IPA", {
      { true, "HEAT", "ADD WATER", "", 63, 0 },
      { true, "MASH IN", "ADD MALT", "", 63.0, 0 },
      { true, "PROTEIN REST", "", "", 63.0, 70 },
      { true, "MALTOSE REST", "", "", 73.0, 5 },
      { true, "SACCRIFICATION", "", "REMOVE MALT", 78.0, 5 },
      { true, "HOPS BOIL", "", "COMPLETE", 99.5, 80 }
    },
    { 70, 55, 40 }
  },
  {
    "Smoked Beer", {
      { true, "HEAT", "ADD WATER", "", 60, 0 },
      { true, "MASH IN", "ADD MALT", "", 63.0, 0 },
      { true, "PROTEIN REST", "", "", 63.0, 20 },
      { true, "MALTOSE REST", "", "", 73.0, 30 },
      { true, "SACCRIFICATION", "", "REMOVE MALT", 78.0, 5 },
      { true, "HOPS BOIL", "", "COMPLETE", 99.5, 80 }
    },
    { 70, 10, 0 }
  },
  {
    "Pilsner", {
      { true, "HEAT", "ADD WATER", "", 38.0, 0 },
      { true, "MASH IN", "ADD MALT", "", 63.0, 0 },
      { true, "PROTEIN REST", "", "", 63.0, 40 },
      { true, "MALTOSE REST", "", "", 73.0, 25 },
      { true, "SACCRIFICATION", "", "REMOVE MALT", 78.0, 10 },
      { true, "HOPS BOIL", "", "COMPLETE", 99.5, 80 }
    },
    { 70, 10, 0 }
  },
  {
    "Wheat Beer", {
      { true, "HEAT", "ADD WATER", "", 50.0, 0 },
      { true, "MASH IN", "ADD MALT", "", 63.0, 0 },
      { true, "PROTEIN REST", "", "", 63.0, 15 },
      { true, "MALTOSE REST", "", "", 73.0, 35 },
      { true, "SACCRIFICATION", "", "REMOVE MALT", 78.0, 15 },
      { true, "HOPS BOIL", "", "COMPLETE", 99.5, 80 }
    },
    { 70, 0, 0 }
  },
  {
    "38 Wheat Beer", {
      { true, "HEAT", "ADD WATER", "", 38.0, 0 },
      { true, "MASH IN", "ADD MALT", "", 62.0, 0 },
      { true, "PROTEIN REST", "", "", 62.0, 30 },
      { true, "MALTOSE REST", "", "", 72.0, 40 },
      { true, "SACCRIFICATION", "", "REMOVE MALT", 78.0, 10 },
      { true, "HOPS BOIL", "", "COMPLETE", 99.5, 80 }
    },
    { 70, 40, 0 }
  },
  {
    "Bohemian Lager", {
      { true, "HEAT", "ADD WATER", "", 62.0, 0 },
      { true, "MASH IN", "ADD MALT", "", 62.0, 0 },
      { true, "PROTEIN REST", "", "", 62.0, 40 },
      { true, "MALTOSE REST", "", "", 72.0, 30 },
      { true, "SACCRIFICATION", "", "REMOVE MALT", 78.0, 10 },
      { true, "HOPS BOIL", "", "COMPLETE", 99.5, 80 }
    },
    { 65, 40, 0 }
  },
  {
    "Eichenbock", {
      { true, "HEAT", "ADD WATER", "", 38.0, 0 },
      { true, "MASH IN", "ADD MALT", "", 65.0, 0 },
      { true, "PROTEIN REST", "", "", 65.0, 30 },
      { true, "MALTOSE REST", "", "", 72.0, 40 },
      { true, "SACCRIFICATION", "", "REMOVE MALT", 78.0, 5 },
      { true, "HOPS BOIL", "", "COMPLETE", 99.5, 90 }
    },
    { 80, 70, 0 }
  },
  {
    "Blackberry Beer", {
      { true, "HEAT", "ADD WATER", "", 60.0, 0 },
      { true, "MASH IN", "ADD MALT", "", 63.0, 0 },
      { true, "PROTEIN REST", "", "", 63.0, 10 },
      { true, "MALTOSE REST", "", "", 72.0, 40 },
      { true, "SACCRIFICATION", "", "REMOVE MALT", 78.0, 10 },
      { true, "HOPS BOIL", "", "COMPLETE", 99.5, 80 }
    },
    { 70, 10, 5 }
  },
  {
    "BavariaMandarina", {
      { true, "HEAT", "ADD WATER", "", 38.0, 0 },
      { true, "MASH IN", "ADD MALT", "", 62.0, 0 },
      { true, "PROTEIN REST", "", "", 62.0, 40 },
      { true, "MALTOSE REST", "", "", 72.0, 25 },
      { true, "SACCRIFICATION", "", "REMOVE MALT", 78.0, 10 },
      { true, "HOPS BOIL", "", "COMPLETE", 99.5, 75 }
    },
    { 65, 40, 0 }
  }
};

#endif
