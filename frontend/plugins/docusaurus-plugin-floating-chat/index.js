/**
 * Plugin to add floating chat widget to all pages
 */

module.exports = function(context, options) {
  return {
    name: 'docusaurus-plugin-floating-chat',
    
    getClientModules() {
      return [require.resolve('./src/floatingChatInjector')];
    },
  };
};