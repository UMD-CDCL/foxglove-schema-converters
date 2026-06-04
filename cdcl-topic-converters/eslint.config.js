const foxglove = require("@foxglove/eslint-plugin");

module.exports = [
  ...foxglove.configs.recommended,
  {
    files: ["**/*.ts", "**/*.tsx"],
    rules: {},
  },
];
