import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function NotFound() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout title={`Page Not Found | ${siteConfig.title}`}>
      <main className="container margin-vert--xl">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1 className="hero__title">Page Not Found</h1>
            <p>We couldn't find the page you were looking for.</p>
            <p>
              <Link to="/">Go to the homepage</Link>
            </p>
            <p>
              <Link to="/docs/intro">Go to the textbook introduction</Link>
            </p>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default NotFound;