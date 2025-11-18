const path = require('node:path');

module.exports = {
  rootDir: path.resolve(__dirname),
  testEnvironment: 'node',
  testTimeout: 60000,
  verbose: false,
  roots: ['<rootDir>/tests'],
  transform: {},
  moduleFileExtensions: ['js', 'json'],
  setupFilesAfterEnv: [],
};
