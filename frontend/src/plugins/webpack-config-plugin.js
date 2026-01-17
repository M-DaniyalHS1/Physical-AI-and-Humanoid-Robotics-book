// @ts-check

/**
 * @param {import('@docusaurus/types').LoadContext} context
 */
module.exports = function createConfigPlugin(context) {
  return {
    name: 'custom-webpack-config',
    
    configureWebpack() {
      return {
        resolve: {
          fallback: {
            process: require.resolve('process/browser'),
          },
        },
        plugins: [
          new (require('webpack')).DefinePlugin({
            'process.env': '{}',
            process: 'process/browser',
          }),
        ],
      };
    },
  };
};